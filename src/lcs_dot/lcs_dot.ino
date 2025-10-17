#include "LedControl.h"
#include <LiquidCrystal.h>
#include <math.h>

// ===================== LCD (Temp) =====================
LiquidCrystal lcd(5, 6, 7, 8, 9, 10);

// ===================== MAX7219 (Time) =====================
// LedControl(DIN, CLK, CS, numDevices)
LedControl lc(4, 2, 3, 1);

// --- timers ---
unsigned long tNow;
unsigned long tLCD = 0, tLCDPeriod = 300;
unsigned long tMx = 0, tMxPeriod = 60; // scroll speed (ms)
unsigned long tSample = 0, tSamplePeriod = 500;

// ===================== Thermistor =====================
const int TEMP_PIN = A0;
const float Rref = 10000.0f; // 10k series resistor
const float R0 = 10000.0f;   // NTC @ 25°C
const float T0K = 298.15f;   // 25°C in K
const float BETA = 3950.0f;
const bool RREF_HIGH_SIDE = true; // 5V->Rref->A0->NTC->GND

float tempC = NAN, tempMin = NAN, tempMax = NAN;

// ===================== Tiny 5x7 font (columns L->R) =====================
// Each glyph: 5 columns, LSB at top (bit0=row0). We’ll center vertically.
const byte FONT_5x7[][5] PROGMEM = {
    // '0'..'9'
    {B0111110, B1000001, B1000011, B1000101, B0111110}, // 0
    {B0000000, B0000010, B1111111, B0000000, B0000000}, // 1
    {B1110010, B1001001, B1001001, B1001001, B1000110}, // 2
    {B0100010, B1000001, B1001001, B1001001, B0110110}, // 3
    {B0001100, B0010100, B0100100, B1111111, B0000100}, // 4
    {B0100111, B1000101, B1000101, B1000101, B0111001}, // 5
    {B0111110, B1001001, B1001001, B1001001, B0110010}, // 6
    {B0000001, B1110001, B0001001, B0000101, B0000011}, // 7
    {B0110110, B1001001, B1001001, B1001001, B0110110}, // 8
    {B0100110, B1001001, B1001001, B1001001, B0111110}, // 9
};
const byte COLON_5x7[5] PROGMEM = {B0000000, B0100100, B0000000, B0000000, B0000000}; // two dots

// Build a column stream like "HH:MM" -> push columns one by one
// Scroll buffer state
char timeStr[6] = "00:00"; // HH:MM + null
int scrollCol = -8;        // start off-screen

// ===================== Utils =====================
template <int N> int analogReadAvg(uint8_t pin) {
    long s = 0;
    for (int i = 0; i < N; i++) {
        s += analogRead(pin);
        delayMicroseconds(200);
    }
    return (int)(s / N);
}

float readTempC() {
    int adc = analogReadAvg<8>(TEMP_PIN);
    if (adc <= 0 || adc >= 1023)
        return NAN;

    float Rtherm = RREF_HIGH_SIDE ? Rref * ((float)adc / (1023.0f - (float)adc))
                                  : Rref * ((1023.0f / (float)adc) - 1.0f);

    float invTK = (1.0f / T0K) + (logf(Rtherm / R0) / BETA);
    return (1.0f / invTK) - 273.15f;
}

// Convert "HH:MM" into a column at logical index k (with 1 col spacing between glyphs)
bool timeColumnAt(int k, byte &colBitsOut) {
    // Each glyph: 5 cols + 1 space = 6. "HH:MM" -> 5 glyphs -> 30 cols total.
    const int glyphCols = 6;
    const int totalCols = 5 * glyphCols; // 30

    if (k < 0 || k >= totalCols) {
        colBitsOut = 0;
        return false;
    }

    int g = k / glyphCols; // glyph index 0..4
    int c = k % glyphCols; // column within glyph 0..5 (5 is space)
    if (c == 5) {
        colBitsOut = 0;
        return true;
    }

    char ch = timeStr[g];
    byte col = 0;

    if (ch >= '0' && ch <= '9') {
        col = pgm_read_byte(&FONT_5x7[ch - '0'][c]);
    } else if (ch == ':') {
        col = pgm_read_byte(&COLON_5x7[c]);
    }

    // We have a 7-pixel-tall glyph; center it in 8 rows by shifting down 0..1.
    // Here we’ll place row0 of glyph at matrix row0 (no shift), and ensure MSB is 0.
    colBitsOut = col & B01111111; // keep 7 bits
    return true;
}

// Push current 8 cols (window) to the matrix at this scroll offset
void renderTimeScroll() {
    // display window spans 8 columns; we shift time string from right->left
    // scrollCol is the leftmost content column relative to matrix col0
    lc.clearDisplay(0);
    for (int x = 0; x < 8; x++) {
        int contentCol = x + scrollCol;
        byte bits = 0;
        timeColumnAt(contentCol, bits);
        lc.setColumn(0, x, bits);
    }
}

// Update `timeStr` from uptime (millis). If you later add RTC, swap here.
void updateTimeString() {
    unsigned long s = millis() / 1000UL;
    unsigned long hh = (s / 3600UL) % 24UL;
    unsigned long mm = (s / 60UL) % 60UL;

    timeStr[0] = '0' + (hh / 10);
    timeStr[1] = '0' + (hh % 10);
    timeStr[2] = ':';
    timeStr[3] = '0' + (mm / 10);
    timeStr[4] = '0' + (mm % 10);
    timeStr[5] = '\0';
}

// ===================== Setup =====================
void setup() {
    analogReference(DEFAULT);
    lcd.begin(16, 2);

    lc.shutdown(0, false);
    lc.setIntensity(0, 8);
    lc.clearDisplay(0);
}

// ===================== Loop =====================
void loop() {
    tNow = millis();

    // --- read/update temperature (every 500 ms) ---
    if (tNow - tSample >= tSamplePeriod) {
        tSample = tNow;
        tempC = readTempC();
        if (!isnan(tempC)) {
            if (isnan(tempMin) || tempC < tempMin)
                tempMin = tempC;
            if (isnan(tempMax) || tempC > tempMax)
                tempMax = tempC;
        }
    }

    // --- LCD refresh (temp + min/max) ---
    if (tNow - tLCD >= tLCDPeriod) {
        tLCD = tNow;
        lcd.setCursor(0, 0);
        if (isnan(tempC)) {
            lcd.print("Temp ERR       ");
        } else {
            lcd.print("Temp         C  ");
            lcd.setCursor(6, 0);
            lcd.print(tempC, 1);
            lcd.print("  ");
        }
        lcd.setCursor(0, 1);
        lcd.print("Min:");
        if (!isnan(tempMin))
            lcd.print((int)round(tempMin));
        else
            lcd.print("--");
        lcd.print("C Max:");
        if (!isnan(tempMax))
            lcd.print((int)round(tempMax));
        else
            lcd.print("--");
        lcd.print("C");
    }

    // --- Matrix: scroll HH:MM ---
    if (tNow - tMx >= tMxPeriod) {
        tMx = tNow;

        // update text about once per second (minutes will tick at 60s)
        static unsigned long lastSec = 0;
        unsigned long sec = tNow / 1000UL;
        if (sec != lastSec) {
            lastSec = sec;
            updateTimeString();
        }

        renderTimeScroll();
        // total content width is 30 cols. Scroll from right to left then loop.
        // Start from +8 (off-screen right) to -30 (off-screen left)
        static const int startCol = 8;
        static const int endCol = -30;
        if (scrollCol > endCol)
            scrollCol--;
        else
            scrollCol = startCol;
    }
}
