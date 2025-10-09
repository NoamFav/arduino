#include "cpg.h"

void readInput() {
    if (!Serial.available())
        return;
    String cmd = Serial.readStringUntil('\n');

    if (cmd.startsWith("amp")) {
        int i = cmd.substring(4, 5).toInt();
        osc[i].targetAmplitude = cmd.substring(6).toInt();

    } else if (cmd.startsWith("off")) {
        int i = cmd.substring(4, 5).toInt();
        osc[i].targetOffset = cmd.substring(6).toInt();

    } else if (cmd.startsWith("freq")) {
        targetFrequency = cmd.substring(5).toFloat();

    } else if (cmd.startsWith("phb")) {
        int i = cmd.substring(4, 5).toInt();
        int j = cmd.substring(6, 7).toInt();
        float val = cmd.substring(8).toFloat();
        osc[i].phaseBias[j] = val;
        osc[j].phaseBias[i] = -val;

    } else if (cmd.startsWith("weight")) {
        w = cmd.substring(7).toFloat();

    } else if (cmd.startsWith("print")) {
        Serial.print("Frequency: ");
        Serial.println(frequency);
        for (size_t i = 0; i < NUM_OSCILLATORS; ++i) {
            Serial.printf("%u: [%.2f, %.2f, %.2f, %.2f, %.2f]\n", i, osc[i].phase, osc[i].amplitude,
                          osc[i].offset, osc[i].pos, osc[i].rateOfPhase);
        }
    }
}
