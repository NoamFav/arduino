// EDMO 2-DoF planar arm: Enhanced controller with improved safety, diagnostics, and features
// Commands (Serial @ 9600, "Newline"):
//   ANG a1,a2    -- or simply "a1,a2" (degrees)  -> direct servo set
//   IK  x y [U|D] -- x,y in cm; U=elbow-up, D=elbow-down (default D)
//   HOME         -- move to home position (0,0)
//   STATUS       -- show current position and servo info
//   LIMITS       -- show workspace limits
//   SMOOTH speed -- set movement speed (1-10, 5=default)
//   HELP         -- show command help

#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <Wire.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---------- Configuration ----------
#define NUM_MOTORS 2
#define LEFTEND -90 // allowed user angle range (deg)
#define RIGHTEND 90
#define MAX_SPEED 10 // movement speed levels
#define DEFAULT_SPEED 5
#define POSITION_TOLERANCE 0.5f // degrees

// Servo PWM limits (from EDMO specifications)
const int SERVOMIN[2] = {78, 78};   // ch0, ch1  (0..4095)
const int SERVOMAX[2] = {472, 494}; // ch0, ch1

// Choose your EDMO generation
#define EDMO_GEN 2 // set 1 for old EDMO, 2 for EDMO 2.0

#if (EDMO_GEN == 1)
// Old EDMO (slides)
const float L1 = 5.40f;  // cm
const float L2 = 10.40f; // cm
const float L3 = 5.00f;  // cm
#else
// EDMO 2.0 (slides)
const float L1 = 6.65f;  // cm
const float L2 = 13.30f; // cm
const float L3 = 6.65f;  // cm
#endif

// Effective 2nd segment assumes tip L3 colinear with link-2
const float RSEG = L2 + L3;

// Servo ↔️ kinematics angle mapping
// θj = THETA_ZERO[j] + SGN[j] * αj   (degrees)
float THETA_ZERO[2] = {90.0f, 0.0f}; // adjust if your neutral differs
int SGN[2] = {+1, -1};               // flip sign if a joint turns opposite

// ---------- Global Variables ----------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// State variables
float currentAngles[NUM_MOTORS] = {0.0f, 0.0f}; // current α1, α2 (deg)
float targetAngles[NUM_MOTORS] = {0.0f, 0.0f};  // target α1, α2 (deg)
int pwmValues[NUM_MOTORS] = {0, 0};             // current PWM values
int movementSpeed = DEFAULT_SPEED;              // movement speed (1-10)
bool isMoving = false;                          // movement state flag

// Communication buffers
char inputBuffer[100];
int bufferIndex = 0;
unsigned long lastMoveTime = 0;
const unsigned long MOVE_INTERVAL = 20; // ms between smooth movement steps

// Error tracking
int errorCount = 0;
unsigned long lastErrorTime = 0;

// ---------- Utility Functions ----------
static inline float deg2rad(float d) {
    return d * 0.017453292519943295f;
}

static inline float rad2deg(float r) {
    return r * 57.29577951308232f;
}

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float absf(float x) {
    return x < 0 ? -x : x;
}

// Linear interpolation
float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

// ---------- Movement Functions ----------
bool isAtTarget() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (absf(currentAngles[i] - targetAngles[i]) > POSITION_TOLERANCE) {
            return false;
        }
    }
    return true;
}

void updateSmoothMovement() {
    if (!isMoving || millis() - lastMoveTime < MOVE_INTERVAL) {
        return;
    }

    lastMoveTime = millis();
    float stepSize = (float)movementSpeed / 100.0f; // Convert speed to step size
    bool allReached = true;

    for (int i = 0; i < NUM_MOTORS; i++) {
        float diff = targetAngles[i] - currentAngles[i];
        if (absf(diff) > POSITION_TOLERANCE) {
            float step = diff * stepSize;
            currentAngles[i] += step;
            allReached = false;
        } else {
            currentAngles[i] = targetAngles[i];
        }
    }

    writeToServos();

    if (allReached) {
        isMoving = false;
        Serial.println(F("Movement complete."));
        printCurrentPosition();
    }
}

void moveToAngles(float a1, float a2, bool smooth = true) {
    targetAngles[0] = clampf(a1, LEFTEND, RIGHTEND);
    targetAngles[1] = clampf(a2, LEFTEND, RIGHTEND);

    if (!smooth) {
        currentAngles[0] = targetAngles[0];
        currentAngles[1] = targetAngles[1];
        writeToServos();
        printCurrentPosition();
    } else {
        isMoving = true;
        lastMoveTime = millis();
    }
}

void writeToServos() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        long pulse = map((long)(currentAngles[i] * 100), LEFTEND * 100, RIGHTEND * 100, SERVOMIN[i],
                         SERVOMAX[i]);
        pulse = clampf(pulse, SERVOMIN[i], SERVOMAX[i]);
        pwmValues[i] = pulse;
        pwm.setPWM(i, 0, (int)pulse);
    }
}

// ---------- Kinematics ----------
bool inverseKinematics(float x, float y, int elbow, float &theta1_deg, float &theta2_deg) {
    float r = sqrtf(x * x + y * y);
    float reachMin = absf(L1 - RSEG);
    float reachMax = L1 + RSEG;
    bool reachable = (r >= reachMin && r <= reachMax && r > 0.001f);

    // Avoid division by zero and numerical issues
    if (r < 0.001f) {
        theta1_deg = 0.0f;
        theta2_deg = 0.0f;
        return false;
    }

    // cos(theta2)
    float c2 = (x * x + y * y - L1 * L1 - RSEG * RSEG) / (2.0f * L1 * RSEG);
    c2 = clampf(c2, -1.0f, 1.0f);

    // choose elbow by signed sin(theta2)
    float s2 = elbow * sqrtf(fmaxf(0.0f, 1.0f - c2 * c2));
    float theta2 = atan2f(s2, c2);

    // theta1 via two-atan2 terms
    float k1 = L1 + RSEG * c2;
    float k2 = RSEG * s2;
    float theta1 = atan2f(y, x) - atan2f(k2, k1);

    theta1_deg = rad2deg(theta1);
    theta2_deg = rad2deg(theta2);

    return reachable;
}

void forwardKinematics(float a1, float a2, float &x, float &y) {
    float theta1 = THETA_ZERO[0] + SGN[0] * a1;
    float theta2 = THETA_ZERO[1] + SGN[1] * a2;

    float t1 = deg2rad(theta1);
    float t12 = deg2rad(theta1 + theta2);

    x = L1 * cosf(t1) + RSEG * cosf(t12);
    y = L1 * sinf(t1) + RSEG * sinf(t12);
}

// ---------- Display Functions ----------
void printHelp() {
    Serial.println(F("\n=== EDMO Enhanced IK/FK Controller ==="));
#if (EDMO_GEN == 1)
    Serial.println(F("EDMO gen: 1   L1=5.40  L2=10.40  L3=5.00 [cm]"));
#else
    Serial.println(F("EDMO gen: 2   L1=6.65  L2=13.30  L3=6.65 [cm]"));
#endif
    Serial.println(F("\nCommands:"));
    Serial.println(F("  ANG a1,a2     (or a1,a2)  -> servo angles (deg)"));
    Serial.println(F("  IK x y [U|D]  -> go to (x,y) cm; U=elbow-up, D=elbow-down"));
    Serial.println(F("  HOME          -> move to home position (0,0)"));
    Serial.println(F("  STATUS        -> show current position and servo info"));
    Serial.println(F("  LIMITS        -> show workspace limits"));
    Serial.println(F("  SMOOTH n      -> set movement speed (1-10, 5=default)"));
    Serial.println(F("  HELP          -> show this help"));

    float reachMin = absf(L1 - RSEG);
    float reachMax = L1 + RSEG;
    Serial.print(F("\nWorkspace: "));
    Serial.print(reachMin, 2);
    Serial.print(F(" to "));
    Serial.print(reachMax, 2);
    Serial.println(F(" cm radius"));
    Serial.println(F("Servo range: -90° to +90°\n"));
}

void printCurrentPosition() {
    float x, y;
    forwardKinematics(currentAngles[0], currentAngles[1], x, y);

    float theta1 = THETA_ZERO[0] + SGN[0] * currentAngles[0];
    float theta2 = THETA_ZERO[1] + SGN[1] * currentAngles[1];

    // CSV format: a1,a2,pwm1,pwm2,θ1,θ2,x_cm,y_cm
    Serial.print(currentAngles[0], 3);
    Serial.print(',');
    Serial.print(currentAngles[1], 3);
    Serial.print(',');
    Serial.print(pwmValues[0]);
    Serial.print(',');
    Serial.print(pwmValues[1]);
    Serial.print(',');
    Serial.print(theta1, 3);
    Serial.print(',');
    Serial.print(theta2, 3);
    Serial.print(',');
    Serial.print(x, 3);
    Serial.print(',');
    Serial.println(y, 3);

    Serial.print(F("Position: α1="));
    Serial.print(currentAngles[0], 2);
    Serial.print(F("°, α2="));
    Serial.print(currentAngles[1], 2);
    Serial.print(F("° | θ1="));
    Serial.print(theta1, 2);
    Serial.print(F("°, θ2="));
    Serial.print(theta2, 2);
    Serial.print(F("° | End: ("));
    Serial.print(x, 2);
    Serial.print(F(", "));
    Serial.print(y, 2);
    Serial.println(F(") cm"));
}

void printStatus() {
    Serial.println(F("=== SYSTEM STATUS ==="));
    printCurrentPosition();
    Serial.print(F("Movement speed: "));
    Serial.println(movementSpeed);
    Serial.print(F("Is moving: "));
    Serial.println(isMoving ? F("YES") : F("NO"));
    Serial.print(F("Error count: "));
    Serial.println(errorCount);

    float reachMin = absf(L1 - RSEG);
    float reachMax = L1 + RSEG;
    float x, y;
    forwardKinematics(currentAngles[0], currentAngles[1], x, y);
    float currentReach = sqrtf(x * x + y * y);

    Serial.print(F("Current reach: "));
    Serial.print(currentReach, 2);
    Serial.print(F(" cm ("));
    Serial.print((currentReach / reachMax) * 100.0f, 1);
    Serial.println(F("% of max)"));
}

void printLimits() {
    float reachMin = absf(L1 - RSEG);
    float reachMax = L1 + RSEG;

    Serial.println(F("=== WORKSPACE LIMITS ==="));
    Serial.print(F("Min reach: "));
    Serial.print(reachMin, 2);
    Serial.println(F(" cm"));
    Serial.print(F("Max reach: "));
    Serial.print(reachMax, 2);
    Serial.println(F(" cm"));
    Serial.print(F("Servo angles: "));
    Serial.print(LEFTEND);
    Serial.print(F("° to "));
    Serial.print(RIGHTEND);
    Serial.println(F("°"));
    Serial.print(F("PWM ranges: CH0["));
    Serial.print(SERVOMIN[0]);
    Serial.print(F("-"));
    Serial.print(SERVOMAX[0]);
    Serial.print(F("], CH1["));
    Serial.print(SERVOMIN[1]);
    Serial.print(F("-"));
    Serial.print(SERVOMAX[1]);
    Serial.println(F("]"));
}

// ---------- Command Parsing ----------
void logError(const char *msg) {
    errorCount++;
    lastErrorTime = millis();
    Serial.print(F("ERROR: "));
    Serial.println(msg);
}

bool startsWithIgnoreCase(const char *s, const char *pre) {
    while (*pre) {
        if (toupper((unsigned char)*s++) != toupper((unsigned char)*pre++))
            return false;
    }
    return true;
}

void replaceChar(char *s, char from, char to) {
    for (; *s; ++s)
        if (*s == from)
            *s = to;
}

void handleAngles(char *s) {
    replaceChar(s, ',', ' ');
    char *tok = strtok(s, " \t");
    float a1 = 0, a2 = 0;
    int cnt = 0;

    while (tok && cnt < 2) {
        if (*tok) {
            float val = atof(tok);
            if (val < LEFTEND - 10 || val > RIGHTEND + 10) {
                logError("Angle out of reasonable range");
                return;
            }
            if (cnt == 0)
                a1 = val;
            else
                a2 = val;
            cnt++;
        }
        tok = strtok(NULL, " \t");
    }

    if (cnt != 2) {
        logError("Need two angles. Example: 15,-20 or ANG 15,-20");
        return;
    }

    moveToAngles(a1, a2);
}

void handleIK(char *s) {
    s += 2; // skip "IK"
    replaceChar(s, ',', ' ');
    char *tok = strtok(s, " \t");
    float x = NAN, y = NAN;
    int elbow = -1; // default elbow-down
    int idx = 0;

    while (tok) {
        if (*tok == '\0') {
            tok = strtok(NULL, " \t");
            continue;
        }
        if (idx == 0)
            x = atof(tok);
        else if (idx == 1)
            y = atof(tok);
        else if (idx == 2) {
            char c = toupper(*tok);
            if (c == 'U')
                elbow = +1;
            else if (c == 'D')
                elbow = -1;
            else
                elbow = (atoi(tok) >= 0) ? +1 : -1;
        }
        idx++;
        tok = strtok(NULL, " \t");
    }

    if (isnan(x) || isnan(y)) {
        logError("IK needs x y [U|D]. Example: IK 12.3 18.0 U");
        return;
    }

    float reachMax = L1 + RSEG;
    float targetReach = sqrtf(x * x + y * y);
    if (targetReach > reachMax + 0.5f) {
        Serial.print(F("WARNING: Target ("));
        Serial.print(x, 2);
        Serial.print(F(", "));
        Serial.print(y, 2);
        Serial.print(F(") is "));
        Serial.print(targetReach, 2);
        Serial.print(F(" cm, exceeds max reach "));
        Serial.print(reachMax, 2);
        Serial.println(F(" cm"));
    }

    float th1, th2;
    bool reachable = inverseKinematics(x, y, elbow, th1, th2);

    float a1 = (th1 - THETA_ZERO[0]) / (float)SGN[0];
    float a2 = (th2 - THETA_ZERO[1]) / (float)SGN[1];

    Serial.print(F("IK: ("));
    Serial.print(x, 2);
    Serial.print(F(", "));
    Serial.print(y, 2);
    Serial.print(F(") cm, elbow "));
    Serial.print(elbow > 0 ? F("UP") : F("DOWN"));
    Serial.print(F(" -> θ1="));
    Serial.print(th1, 2);
    Serial.print(F("°, θ2="));
    Serial.print(th2, 2);
    Serial.print(F("° | Reachable: "));
    Serial.println(reachable ? F("YES") : F("NO (clamped)"));

    moveToAngles(a1, a2);
}

void handleCommand(char *line) {
    // Trim leading spaces
    while (*line == ' ' || *line == '\t')
        line++;

    if (strlen(line) == 0)
        return;

    if (startsWithIgnoreCase(line, "HELP")) {
        printHelp();
    } else if (startsWithIgnoreCase(line, "STATUS")) {
        printStatus();
    } else if (startsWithIgnoreCase(line, "LIMITS")) {
        printLimits();
    } else if (startsWithIgnoreCase(line, "HOME")) {
        Serial.println(F("Moving to home position..."));
        moveToAngles(0, 0);
    } else if (startsWithIgnoreCase(line, "SMOOTH")) {
        char *p = line + 6;
        while (*p == ' ' || *p == '\t')
            p++;
        int speed = atoi(p);
        if (speed >= 1 && speed <= MAX_SPEED) {
            movementSpeed = speed;
            Serial.print(F("Movement speed set to "));
            Serial.println(speed);
        } else {
            logError("Speed must be 1-10");
        }
    } else if (startsWithIgnoreCase(line, "IK")) {
        handleIK(line);
    } else if (startsWithIgnoreCase(line, "ANG")) {
        char *p = line + 3;
        handleAngles(p);
    } else {
        // Try to parse as direct angles
        handleAngles(line);
    }
}

// ---------- Arduino Setup & Loop ----------
void setup() {
    Serial.begin(9600);
    pwm.begin();
    pwm.setPWMFreq(50);
    delay(100);

    // Initialize to home position
    moveToAngles(0, 0, false);

    while (!Serial) {
        delay(10);
    }

    Serial.println(F("EDMO Enhanced Controller Ready!"));
    printHelp();
    Serial.println(F("Type STATUS to see current position."));
}

void loop() {
    // Handle smooth movement
    updateSmoothMovement();

    // Process serial input
    while (Serial.available()) {
        char c = Serial.read();
        if (c != '\n' && c != '\r') {
            if (bufferIndex < sizeof(inputBuffer) - 1) {
                inputBuffer[bufferIndex++] = c;
            }
        } else if (bufferIndex > 0) {
            inputBuffer[bufferIndex] = '\0';
            bufferIndex = 0;
            handleCommand(inputBuffer);
        }
    }

    // Small delay to prevent overwhelming the serial
    delay(1);
}
