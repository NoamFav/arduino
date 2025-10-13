#include "cpg.h"
#include "io.h"

static unsigned long prevMillis = 0;
static constexpr unsigned long kTimeStepMs = 10;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
    }
    initServos();
}

void loop() {
    const unsigned long now = millis();
    if (now - prevMillis >= kTimeStepMs) {
        const unsigned long dt = now - prevMillis;
        prevMillis = now;

        readInput();
        updateCPG(dt);
        setServos();

        emitTelemetry(now);
    }
}
