#include "config.h"
#include "io.h"
#include "pid.h"
#include "serial_cmd.h"
#include "telemetry.h"

PIDController pid_controller;
PerformanceMetrics metrics;
float target_position = 512.0f;
bool telemetry_enabled = true;

void setup() {
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);

    Serial.begin(115200);
    setMotorOutput(0);

    Serial.println("PID Motor Controller Ready");
    Serial.println("Type 'help' for commands");
    printHelp();
}

void loop() {
    static uint32_t last = 0;
    uint32_t now = millis();

    if (now - last >= SAMPLE_TIME_MS) {
        last += SAMPLE_TIME_MS;
        float pos = readPotentiometerPosition();
        float err = target_position - pos;
        float out = executePIDStep(pid_controller, err);
        setMotorOutput(out);
        updatePerformanceMetrics(metrics, err);
        if (telemetry_enabled)
            sendStatusUpdate(pos, err, out, pid_controller, target_position);
    }
    processSerialInput();
}
