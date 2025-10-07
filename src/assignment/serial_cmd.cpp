// serial_cmd.cpp
#include "serial_cmd.h"
#include "telemetry.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

static char serial_buffer[64];
static size_t serial_buffer_length = 0;

// Returns true if s starts with prefix p
static inline bool startsWith(const char *s, const char *p) {
    while (*p) {
        if (*s++ != *p++)
            return false;
    }
    return true;
}

// Skip ASCII spaces/tabs returns first non-WS char
static inline const char *skipWS(const char *s) {
    while (*s == ' ' || *s == '\t')
        ++s;
    return s;
}

// Parse a simple decimal float
// Sets *endptr to first unparsed char
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

// Tokenize in place into up to 4 tokens delimiters: ',', ';', ' '
// Returns token count writes '\0' terminators into s
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
        if (*s) {
            *s++ = 0;
        }
    }
    return n;
}

// Parse and execute a single command line
// Recognizes the commands listed in the file header
// Uses a 64-byte local copy (buf) overlong input is truncated safely.
void processSerialCommand(const char *in) {
    static char buf[64];
    size_t L = 0;
    for (; L < sizeof(buf) - 1 && in[L]; ++L)
        buf[L] = in[L];
    buf[L] = 0;
    char *cmd = buf;
    while (*cmd == ' ' || *cmd == '\t')
        ++cmd;
    if (!*cmd)
        return;

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        printHelp();
        return;
    }
    if (!strcmp(cmd, "status")) {
        Serial.println("=== Current Settings ===");
        Serial.print("Target: ");
        Serial.println(target_position, 1);
        Serial.print("PID Gains - Kp: ");
        Serial.print(pid_controller.kp, 3);
        Serial.print(", Ki: ");
        Serial.print(pid_controller.ki, 3);
        Serial.print(", Kd: ");
        Serial.println(pid_controller.kd, 3);
        Serial.println("======================");
        return;
    }
    if (startsWith(cmd, "kp=")) {
        pid_controller.kp = parseFloat(cmd + 3);
        Serial.print("ACK kp=");
        Serial.println(pid_controller.kp, 3);
        return;
    }
    if (startsWith(cmd, "ki=")) {
        pid_controller.ki = parseFloat(cmd + 3);
        Serial.print("ACK ki=");
        Serial.println(pid_controller.ki, 3);
        return;
    }
    if (startsWith(cmd, "kd=")) {
        pid_controller.kd = parseFloat(cmd + 3);
        Serial.print("ACK kd=");
        Serial.println(pid_controller.kd, 3);
        return;
    }
    if (startsWith(cmd, "sp=") || startsWith(cmd, "sp ")) {
        pid_controller.integral_sum = 0.0f;
        target_position = constrain(parseFloat(cmd + 3), 0.0f, POT_RANGE);
        Serial.print("ACK sp=");
        Serial.println(target_position, 1);
        return;
    }
    char *tok[4];
    int n = split4(cmd, tok);
    if (n == 4) {
        float a = parseFloat(tok[0]), b = parseFloat(tok[1]), c = parseFloat(tok[2]),
              d = parseFloat(tok[3]);
        target_position = constrain(a, 0.0f, POT_RANGE);
        pid_controller.kp = b;
        pid_controller.ki = c;
        pid_controller.kd = d;
        pid_controller.integral_sum = 0.0f;
        Serial.println("ACK batch");
        return;
    }
    if ((*cmd == '+' || *cmd == '-' || (*cmd >= '0' && *cmd <= '9'))) {
        pid_controller.integral_sum = 0.0f;
        target_position = constrain(parseFloat(cmd), 0.0f, POT_RANGE);
        Serial.print("ACK sp=");
        Serial.println(target_position, 1);
        return;
    }
    if (!strcmp(cmd, "tele=1")) {
        telemetry_enabled = true;
        Serial.println("ACK tele=1");
        return;
    }
    if (!strcmp(cmd, "tele=0")) {
        telemetry_enabled = false;
        Serial.println("ACK tele=0");
        return;
    }
    Serial.println("Unknown command. Type 'help'.");
}

// Consume bytes from Serial until '\n', on each full line, calls
// processSerialCommand(). Ignores '\r', Maintains serial_buffer_length
void processSerialInput() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r')
            continue;
        if (c == '\n') {
            serial_buffer[min(serial_buffer_length, sizeof(serial_buffer) - 1)] = '\0';
            processSerialCommand(serial_buffer);
            serial_buffer_length = 0;
        } else if (serial_buffer_length < sizeof(serial_buffer) - 1) {
            serial_buffer[serial_buffer_length++] = c;
        }
    }
}
