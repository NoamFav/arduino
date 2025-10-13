#include "cpg.h"
#include <Arduino.h>
#include <string.h>
static char serial_buffer[64];
static size_t serial_buffer_length = 0;

static inline bool startsWith(const char *s, const char *p) {
    while (*p)
        if (*s++ != *p++)
            return false;
    return true;
}

static inline const char *skipWS(const char *s) {
    while (*s == ' ' || *s == '\t')
        ++s;
    return s;
}

static float parseFloat(const char *s, const char **endptr = nullptr) {
    s = skipWS(s);
    bool neg = false;
    if (*s == '+' || *s == '-') {
        neg = (*s == '-');
        ++s;
    }
    unsigned long ip = 0;
    while (*s >= '0' && *s <= '9') {
        ip = ip * 10 + (*s - '0');
        ++s;
    }
    unsigned long fp = 0, denom = 1;
    if (*s == '.') {
        ++s;
        while (*s >= '0' && *s <= '9') {
            fp = fp * 10 + (*s - '0');
            denom *= 10;
            ++s;
        }
    }
    float v = (float)ip + (denom > 1 ? (float)fp / (float)denom : 0.0f);
    if (endptr)
        *endptr = s;
    return neg ? -v : v;
}

static int split4(char *s, char *out[4]) {
    int n = 0;
    while (*s && n < 4) {
        while (*s == ' ' || *s == '\t')
            ++s;
        if (!*s)
            break;
        out[n++] = s;
        while (*s && *s != ',' && *s != ';' && *s != ' ')
            ++s;
        if (*s)
            *s++ = 0;
    }
    return n;
}

static void processSerialCommand(const char *in) {
    static char buf[64];
    size_t L = 0;
    for (; L < sizeof(buf) - 1 && in[L]; ++L)
        buf[L] = in[L];
    buf[L] = 0;

    char *cmd = buf;
    while (*cmd == ' ' || *cmd == '\t')
        ++cmd;
    for (char *p = cmd; *p; ++p)
        if (*p >= 'A' && *p <= 'Z')
            *p = char(*p - 'A' + 'a');

    if (!*cmd)
        return;

    Serial.print(F("> "));
    Serial.println(cmd);

    char *tok[4];
    int n = split4(cmd, tok);

    // amp <i> <deg>
    if (n == 3 && !strcmp(tok[0], "amp")) {
        int i = atoi(tok[1]);
        double v = atof(tok[2]);
        if (i >= 0 && (size_t)i < NUM_OSCILLATORS) {
            v = constrain(v, 0.0, 90.0);
            osc[i].targetAmplitude = v;
            Serial.println(F("OK amp"));
        } else
            Serial.println(F("ERR amp index"));
        return;
    }

    // off <i> <deg>
    if (n == 3 && !strcmp(tok[0], "off")) {
        int i = atoi(tok[1]);
        double v = atof(tok[2]);
        if (i >= 0 && (size_t)i < NUM_OSCILLATORS) {
            v = constrain(v, MIN_ANGLE, MAX_ANGLE);
            osc[i].targetOffset = v;
            Serial.println(F("OK off"));
        } else
            Serial.println(F("ERR off index"));
        return;
    }

    // freq <hz>
    if (n == 2 && !strcmp(tok[0], "freq")) {
        double f = atof(tok[1]);
        targetFrequency = f < 0.0 ? 0.0 : f;
        Serial.println(F("OK freq"));
        return;
    }
    // phb <i> <j> <rad>
    if (n == 4 && !strcmp(tok[0], "phb")) {
        int i = atoi(tok[1]);
        int j = atoi(tok[2]);
        double v = atof(tok[3]);
        if (i >= 0 && j >= 0 && (size_t)i < NUM_OSCILLATORS && (size_t)j < NUM_OSCILLATORS) {
            osc[i].phaseBias[j] = v;
            osc[j].phaseBias[i] = -v;
            Serial.println(F("OK phb"));
        } else
            Serial.println(F("ERR phb index"));
        return;
    }
    if (n == 2 && !strcmp(tok[0], "tele")) {
        telemetry_enabled = (atoi(tok[1]) != 0);
        Serial.println(telemetry_enabled ? F("OK tele=1") : F("OK tele=0"));
        return;
    }

    if (!strcmp(tok[0], "snap")) {
        for (size_t k = 0; k < NUM_OSCILLATORS; ++k) {
            osc[k].amplitude = osc[k].targetAmplitude;
            osc[k].offset = osc[k].targetOffset;
        }
        frequency = targetFrequency;
        Serial.println(F("OK snap"));
        return;
    }

    // weight <w>
    if (n == 2 && !strcmp(tok[0], "weight")) {
        w = atof(tok[1]);
        Serial.println(F("OK weight"));
        return;
    }

    // ka <val>  (amplitude adaptation rate)
    if (n == 2 && !strcmp(tok[0], "ka")) {
        a = atof(tok[1]);
        Serial.print(F("OK ka="));
        Serial.println(a, 3);
        return;
    }

    // presets
    if (n >= 2 && !strcmp(tok[0], "preset")) {
        if (!strcmp(tok[1], "inphase")) {
            osc[0].phaseBias[1] = 0.0;
            osc[1].phaseBias[0] = -0.0;
            w = 0.03;
            Serial.println(F("OK preset inphase"));
            return;
        }
        if (!strcmp(tok[1], "antiphase")) {
            osc[0].phaseBias[1] = PI;
            osc[1].phaseBias[0] = -PI;
            w = 0.03;
            Serial.println(F("OK preset antiphase"));
            return;
        }
        if (!strcmp(tok[1], "quarter")) {
            const double q = PI / 2;
            osc[0].phaseBias[1] = q;
            if (!strcmp(tok[0], "stop")) {
                for (size_t k = 0; k < NUM_OSCILLATORS; ++k) {
                    osc[k].targetAmplitude = 0.0;
                    osc[k].amplitude = 0.0; // instant
                }
                targetFrequency = 0.0;
                frequency = 0.0;
                Serial.println(F("OK stop"));
                return;
            }
            osc[1].phaseBias[0] = -q;
            w = 0.03;
            Serial.println(F("OK preset quarter"));
            return;
        }
        Serial.println(F("ERR preset arg"));
        return;
    }

    // stop
    if (!strcmp(tok[0], "stop")) {
        for (size_t k = 0; k < NUM_OSCILLATORS; ++k) {
            osc[k].targetAmplitude = 0.0;
            osc[k].amplitude = 0.0;
            osc[k].targetOffset = 0.0;
            osc[k].offset = 0.0;
        }
        targetFrequency = 0.0;
        frequency = 0.0;
        Serial.println(F("OK stop (parked at 0°)"));
        return;
    }

    // stride <deg>
    if (n == 2 && !strcmp(tok[0], "stride")) {
        gait_stride_deg = constrain(atof(tok[1]), 0.0, 45.0);
        applyWalkGait();
        Serial.println(F("OK stride"));
        return;
    }

    // speed <hz>
    if (n == 2 && !strcmp(tok[0], "speed")) {
        double f = atof(tok[1]);
        gait_speed_hz = (f < 0.0) ? 0.0 : f;
        applyWalkGait();
        Serial.println(F("OK speed"));
        return;
    }

    // bias <deg>
    if (n == 2 && !strcmp(tok[0], "bias")) {
        gait_bias_deg = constrain(atof(tok[1]), -15.0, 15.0);
        applyWalkGait();
        Serial.println(F("OK bias"));
        return;
    }

    // turn <rad>
    if (n == 2 && !strcmp(tok[0], "turn")) {
        gait_turn_rad = constrain(atof(tok[1]), -0.5, 0.5);
        applyWalkGait();
        Serial.println(F("OK turn"));
        return;
    }

    // walk (preset): anti-phase, default stride/speed
    if (!strcmp(tok[0], "walk")) {
        applyWalkGait();
        Serial.println(F("OK walk"));
        return;
    }

    // print
    if (!strcmp(tok[0], "print")) {
        Serial.print(F("frequency="));
        Serial.println(frequency, 4);
        for (size_t k = 0; k < NUM_OSCILLATORS; ++k) {
            Serial.print(k);
            Serial.print(F(": phase="));
            Serial.print(osc[k].phase, 3);
            Serial.print(F(" amp="));
            Serial.print(osc[k].amplitude, 2);
            Serial.print(F(" off="));
            Serial.print(osc[k].offset, 2);
            Serial.print(F(" pos="));
            Serial.print(osc[k].pos, 2);
            Serial.print(F(" dphi="));
            Serial.println(osc[k].rateOfPhase, 3);
        }
        return;
    }

    Serial.println(F("ERR unknown cmd (amp/off/freq/phb/weight/stop/print/preset ...)"));
}

void readInput() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r')
            continue;
        if (c == '\n') {
            serial_buffer[(serial_buffer_length < sizeof(serial_buffer) - 1)
                              ? serial_buffer_length
                              : sizeof(serial_buffer) - 1] = '\0';
            processSerialCommand(serial_buffer);
            serial_buffer_length = 0;
        } else if (serial_buffer_length < sizeof(serial_buffer) - 1) {
            serial_buffer[serial_buffer_length++] = c;
        }
    }
}
