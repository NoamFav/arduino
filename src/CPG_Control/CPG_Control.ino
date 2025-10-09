#include "cpg.h"
#include "io.h"

unsigned long prevMillis = 0;
const unsigned long timeStep = 10;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
    }
    initServos();
}

void loop() {
    unsigned long now = millis();
    if (now - prevMillis >= timeStep) {
        unsigned long dt = now - prevMillis;
        prevMillis = now;

        readInput();
        updateCPG(dt);
        setServos();
    }
}
