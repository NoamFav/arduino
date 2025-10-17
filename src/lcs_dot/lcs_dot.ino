#include "LedControl.h"
#include <LiquidCrystal.h>

// --- MAX7219 setup ---
LedControl lc = LedControl(5, 3, 4, 1); // DIN=5, CLK=3, CS=4
unsigned long delaytime1 = 500;
unsigned long delaytime2 = 50;

// --- LCD setup ---
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// ------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------
void setup() {
    // --- Matrix setup ---
    lc.shutdown(0, false);
    lc.setIntensity(0, 8);
    lc.clearDisplay(0);

    // --- LCD setup ---
    lcd.begin(16, 2);
    lcd.print("Hello, World!");
}

// ------------------------------------------------------------------
// MATRIX FUNCTIONS
// ------------------------------------------------------------------
void writeArduinoOnMatrix() {
    byte a[5] = {B01111110, B10001000, B10001000, B10001000, B01111110};
    byte r[5] = {B00010000, B00100000, B00100000, B00010000, B00111110};
    byte d[5] = {B11111110, B00010010, B00100010, B00100010, B00011100};
    byte u[5] = {B00111110, B00000100, B00000010, B00000010, B00111100};
    byte i[5] = {B00000000, B00000010, B10111110, B00100010, B00000000};
    byte n[5] = {B00011110, B00100000, B00100000, B00010000, B00111110};
    byte o[5] = {B00011100, B00100010, B00100010, B00100010, B00011100};

    byte *letters[] = {a, r, d, u, i, n, o};
    for (int l = 0; l < 7; l++) {
        for (int row = 0; row < 5; row++)
            lc.setRow(0, row, letters[l][row]);
        delay(delaytime1);
    }
    lc.clearDisplay(0);
}

void rows() {
    for (int row = 0; row < 8; row++) {
        delay(delaytime2);
        lc.setRow(0, row, B10100000);
        delay(delaytime2);
        lc.setRow(0, row, 0);
        for (int i = 0; i < row; i++) {
            delay(delaytime2);
            lc.setRow(0, row, B10100000);
            delay(delaytime2);
            lc.setRow(0, row, 0);
        }
    }
}

void columns() {
    for (int col = 0; col < 8; col++) {
        delay(delaytime2);
        lc.setColumn(0, col, B00100000);
        delay(delaytime2);
        lc.setColumn(0, col, 0);
        for (int i = 0; i < col; i++) {
            delay(delaytime2);
            lc.setColumn(0, col, B00100000);
            delay(delaytime2);
            lc.setColumn(0, col, 0);
        }
    }
}

void single() {
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            delay(delaytime2);
            lc.setLed(0, row, col, true);
            delay(delaytime2);
            for (int i = 0; i < col; i++) {
                lc.setLed(0, row, col, false);
                delay(delaytime2);
                lc.setLed(0, row, col, true);
                delay(delaytime2);
            }
        }
    }
}

// ------------------------------------------------------------------
// LOOP
// ------------------------------------------------------------------
void loop() {
    // --- LCD updates ---
    lcd.setCursor(0, 1);
    lcd.print(millis() / 1000);

    // --- Matrix animations ---
    writeArduinoOnMatrix();
    rows();
    columns();
    single();
}
