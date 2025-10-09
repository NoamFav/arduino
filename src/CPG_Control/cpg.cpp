#include "cpg.h"
#include "io.h"

const uint16_t SERVOMIN[NUM_OSCILLATORS] = {78, 78, 78};
const uint16_t SERVOMAX[NUM_OSCILLATORS] = {465, 465, 465};

double frequency = 0.5, targetFrequency = 0.5, rateOfFrequency = 0.0;
double w = 0.025, a = 1.0, c = 0.5;

Oscillator osc[NUM_OSCILLATORS] = {{0, 30, 30, 90, 90, 0, 0, 0, 0, 0, {0, PI, 0}, {0, 1, 0}},
                                   {0, 30, 30, 90, 90, 0, 0, 0, 0, 0, {-PI, 0, PI}, {1, 0, 1}},
                                   {0, 30, 30, 90, 90, 0, 0, 0, 0, 0, {0, -PI, 0}, {0, 1, 0}}};

void updateCPG(unsigned long dt) {
    const double delta = dt / 1000.0;

    // frequency adaptation
    rateOfFrequency = c * (targetFrequency - frequency);
    frequency += rateOfFrequency * delta;

    // compute rates
    for (size_t i = 0; i < NUM_OSCILLATORS; ++i) {
        osc[i].rateOfAmplitude = a * (osc[i].targetAmplitude - osc[i].amplitude);
        osc[i].rateOfOffset = c * (osc[i].targetOffset - osc[i].offset);

        double sum = 0;
        for (size_t j = 0; j < NUM_OSCILLATORS; ++j)
            sum += osc[i].coupling[j] * w * sin(osc[j].phase - osc[i].phase - osc[i].phaseBias[j]);
        osc[i].rateOfPhase = 2 * PI * frequency + sum;
    }

    // integrate
    for (size_t i = 0; i < NUM_OSCILLATORS; ++i) {
        osc[i].amplitude += osc[i].rateOfAmplitude * delta;
        osc[i].offset += osc[i].rateOfOffset * delta;
        osc[i].phase += osc[i].rateOfPhase * delta;
        osc[i].pos = osc[i].offset + osc[i].amplitude * sin(osc[i].phase);
    }
}
