// ----- Pins -----// Feather: 3.3V logic
// M1 on pin 20 (SDA) is OK if you're NOT using I2C.
// E1 must be a PWM-capable pin on your Feather; D9 is PWM on common Feathers.

#define E1 9  // PWM speed pin
#define M1 20 // DIR pin (SDA) -- has pull-up to 3.3V on most Feathers

const uint8_t SPEED = 200;    // 0..255
const uint16_t SEC_MS = 80;   // second tick
const uint16_t MIN_MS = 180;  // minute tick
const uint16_t HOUR_MS = 120; // each pulse in hour double
const uint16_t HOUR_GAP = 120;

uint8_t h = 0, m = 0, s = 0;
unsigned long t0 = 0;

void setup() {
    pinMode(E1, OUTPUT);
    pinMode(M1, OUTPUT);

    // Pick a default direction. Because SDA has a pull-up, explicitly drive it.
    digitalWrite(M1, HIGH); // try HIGH first; flip to LOW if motor spins wrong way
    analogWrite(E1, 0);
}

void pulse(uint16_t ms) {
    analogWrite(E1, SPEED);
    delay(ms);
    analogWrite(E1, 0);
}

void loop() {
    unsigned long now = millis();
    if (now - t0 >= 1000UL) {
        t0 += 1000UL;

        // increment clock
        if (++s == 60) {
            s = 0;
            if (++m == 60) {
                m = 0;
                if (++h == 24)
                    h = 0;
            }
        }

        // patterns
        if (s == 0 && m == 0) { // new hour: double pulse
            pulse(HOUR_MS);
            delay(HOUR_GAP);
            pulse(HOUR_MS);
        } else if (s == 0) { // new minute: longer
            pulse(MIN_MS);
        } else { // second: short
            pulse(SEC_MS);
        }
    }
}
