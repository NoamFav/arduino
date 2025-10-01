// telemetry.cpp
#include "telemetry.h"
#include <Arduino.h>
#include <math.h>

void sendStatusUpdate(float pos, float err, float out, const PIDController &pid, float target) {
    static uint32_t n = 0;
    n++;
    switch (OUTPUT_FORMAT) {
    case OUTPUT_MODE_PLOTTER:
        Serial.print(target, 2);
        Serial.print(',');
        Serial.print(pos, 2);
        Serial.print(',');
        Serial.println(out, 1);
        break;
    case OUTPUT_MODE_VERBOSE:
        Serial.print("Target: ");
        Serial.print(target, 1);
        Serial.print(" | Position: ");
        Serial.print(pos, 1);
        Serial.print(" | Error: ");
        Serial.print(err, 1);
        Serial.print(" | Output: ");
        Serial.print(out, 1);
        Serial.print(" | PID(");
        Serial.print(pid.kp, 3);
        Serial.print(",");
        Serial.print(pid.ki, 3);
        Serial.print(",");
        Serial.print(pid.kd, 3);
        Serial.println(")");
        break;
    case OUTPUT_MODE_JSON:
        Serial.print("{\"update\":");
        Serial.print(n);
        Serial.print(",\"time\":");
        Serial.print(millis());
        Serial.print(",\"target\":");
        Serial.print(target, 2);
        Serial.print(",\"position\":");
        Serial.print(pos, 2);
        Serial.print(",\"error\":");
        Serial.print(err, 2);
        Serial.print(",\"output\":");
        Serial.print(out, 1);
        Serial.print(",\"pid\":{\"kp\":");
        Serial.print(pid.kp, 3);
        Serial.print(",\"ki\":");
        Serial.print(pid.ki, 3);
        Serial.print(",\"kd\":");
        Serial.print(pid.kd, 3);
        Serial.print(",\"integral\":");
        Serial.print(pid.integral_sum, 2);
        Serial.println("}}");
        break;
    }
}

void updatePerformanceMetrics(PerformanceMetrics &m, float error) {
    m.loop_count++;
    m.error_sum += fabsf(error);
    m.avg_error = m.error_sum / m.loop_count;
    if (fabsf(error) > m.max_error)
        m.max_error = fabsf(error);

    uint32_t now = millis();
    if (now - m.last_report_time >= PerformanceMetrics::REPORT_INTERVAL_MS) {
        if (OUTPUT_FORMAT == OUTPUT_MODE_VERBOSE) {
            Serial.println("=== Performance Report ===");
            Serial.print("Updates: ");
            Serial.print(m.loop_count);
            Serial.print(" | Avg Error: ");
            Serial.print(m.avg_error, 2);
            Serial.print(" | Max Error: ");
            Serial.print(m.max_error, 2);
            Serial.print(" | Loop Rate: ");
            Serial.print(m.loop_count * 1000.0f / PerformanceMetrics::REPORT_INTERVAL_MS, 1);
            Serial.println(" Hz");
            Serial.println("========================");
        }
        m.last_report_time = now;
        m.max_error = 0.0f;
        m.error_sum = 0.0f;
        m.loop_count = 0;
    }
}

void printHelp() {
    Serial.println("=== PID Motor Controller Help ===");
    Serial.println("Commands:");
    Serial.println("  123          - Set target position");
    Serial.println("  sp=123       - Set target position");
    Serial.println("  kp=1.0 | ki=0.25 | kd=0.02");
    Serial.println("  600,1.0,0.25,0.02 - Set target,kp,ki,kd");
    Serial.println("  help | status");
    Serial.println("Range: Position 0-1023");
    Serial.println("================================");
}
