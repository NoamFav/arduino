#define BLUE 3
#define GREEN 5
#define RED 6

void setup() {
    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);
}

void setColor(int r, int g, int b) {
    int brightness = 20;
    analogWrite(RED, (r * brightness) / 255);
    analogWrite(GREEN, (g * brightness) / 255);
    analogWrite(BLUE, (b * brightness) / 255);
}

void loop() {
    if (millis() < 10000) {
        for (int g = 0; g <= 255; g += 5) {
            setColor(255, g, 0);
            delay(1);
        } // Red → Yellow
        for (int r = 255; r >= 0; r -= 5) {
            setColor(r, 255, 0);
            delay(1);
        } // Yellow → Green
        for (int b = 0; b <= 255; b += 5) {
            setColor(0, 255, b);
            delay(1);
        } // Green → Cyan
        for (int g = 255; g >= 0; g -= 5) {
            setColor(0, g, 255);
            delay(1);
        } // Cyan → Blue
        for (int r = 0; r <= 255; r += 5) {
            setColor(r, 0, 255);
            delay(1);
        } // Blue → Magenta
        for (int b = 255; b >= 0; b -= 5) {
            setColor(255, 0, b);
            delay(1);
        } // Magenta → Red
    }
}
