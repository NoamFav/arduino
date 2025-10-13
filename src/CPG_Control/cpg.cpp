#include "cpg.h"
#include "io.h"

const uint16_t SERVOMIN[NUM_OSCILLATORS] = {100, 100};
const uint16_t SERVOMAX[NUM_OSCILLATORS] = {600, 600};

bool telemetry_enabled = false;
static unsigned long lastTeleMs = 0;
double frequency = 0.5, targetFrequency = 0.5, rateOfFrequency = 0.0;
double w = 0.04, a = 0.6, c = 0.4;

Oscillator osc[NUM_OSCILLATORS] = {
    [] {
        Oscillator o;
        o.phaseBias[0] = 0.0;
        o.coupling[0] = 0.0;
        o.phaseBias[1] = PI;
        o.coupling[1] = 1.0;
        return o;
    }(),
    [] {
        Oscillator o;
        o.phaseBias[0] = -PI;
        o.coupling[0] = 1.0;
        o.phaseBias[1] = 0.0;
        o.coupling[1] = 0.0;
        return o;
    }(),
};

double gait_speed_hz = 0.6;
double gait_stride_deg = 18.0;
double gait_bias_deg = 0.0;
double gait_turn_rad = 0.0;

// define the function
void applyWalkGait() {
    const double pb = PI + gait_turn_rad;
    osc[0].phaseBias[1] = pb;
    osc[1].phaseBias[0] = -pb;

    osc[0].targetAmplitude = osc[1].targetAmplitude = gait_stride_deg;
    osc[0].targetOffset = +gait_bias_deg;
    osc[1].targetOffset = -gait_bias_deg;
    targetFrequency = gait_speed_hz;
}

void updateCPG(unsigned long dt) {
    const double delta = dt / 1000.0;

    rateOfFrequency = c * (targetFrequency - frequency);
    frequency += rateOfFrequency * delta;

    for (size_t i = 0; i < NUM_OSCILLATORS; ++i) {
        auto &oi = osc[i];
        oi.rateOfAmplitude = a * (oi.targetAmplitude - oi.amplitude);
        oi.rateOfOffset = c * (oi.targetOffset - oi.offset);

        double sum = 0.0;
        for (size_t j = 0; j < NUM_OSCILLATORS; ++j) {
            if (i == j)
                continue;
            sum += oi.coupling[j] * w * sin(osc[j].phase - oi.phase - oi.phaseBias[j]);
        }
        oi.rateOfPhase = 2.0 * PI * frequency + sum;
    }

    for (size_t i = 0; i < NUM_OSCILLATORS; ++i) {
        auto &oi = osc[i];
        oi.amplitude += oi.rateOfAmplitude * delta;
        oi.offset += oi.rateOfOffset * delta;
        oi.phase += oi.rateOfPhase * delta;

        oi.pos = oi.offset + oi.amplitude * sin(oi.phase);
        if (oi.pos < MIN_ANGLE)
            oi.pos = MIN_ANGLE;
        if (oi.pos > MAX_ANGLE)
            oi.pos = MAX_ANGLE;
    }
}

void emitTelemetry(unsigned long now_ms) {
    if (!telemetry_enabled)
        return;
    if (now_ms - lastTeleMs < 20)
        return;
    lastTeleMs = now_ms;

    Serial.print(F("pos0:"));
    Serial.print(osc[0].pos, 3);
    Serial.print(' ');
    Serial.print(F("pos1:"));
    Serial.print(osc[1].pos, 3);
    Serial.print(' ');
    Serial.print(F("tgt0:"));
    Serial.print(osc[0].targetOffset + osc[0].targetAmplitude * sin(osc[0].phase), 3);
    Serial.print(' ');
    Serial.print(F("tgt1:"));
    Serial.print(osc[1].targetOffset + osc[1].targetAmplitude * sin(osc[1].phase), 3);
    Serial.print(' ');
    Serial.print(F("off0:"));
    Serial.print(osc[0].offset, 3);
    Serial.print(' ');
    Serial.print(F("off1:"));
    Serial.print(osc[1].offset, 3);
    Serial.print(' ');
    Serial.print(F("amp0:"));
    Serial.print(osc[0].amplitude, 3);
    Serial.print(' ');
    Serial.print(F("amp1:"));
    Serial.print(osc[1].amplitude, 3);
    Serial.print(' ');
    Serial.print(F("freq:"));
    Serial.print(frequency, 3);
    Serial.print(' ');
    Serial.print(F("ka:"));
    Serial.print(a, 3);
    Serial.print(' ');
    Serial.print(F("kc:"));
    Serial.print(c, 3);
    Serial.println();
}
