// io.cpp
#include "io.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

float mapPotentiometerValue(int raw) {
    long v = (long)(raw - POT_RAW_MIN) * 1023L / (POT_RAW_MAX - POT_RAW_MIN);
    if (v < 0)
        v = 0;
    if (v > 1023)
        v = 1023;
    return (float)v;
}

float readPotentiometerPosition() {
    static float filtered = 0.0f;
    if (filtered == 0.0f)
        filtered = mapPotentiometerValue(analogRead(POTENTIOMETER_PIN));
    int raw = analogRead(POTENTIOMETER_PIN);
    float mapped = mapPotentiometerValue(raw);
    filtered += SMOOTHING_ALPHA * (mapped - filtered);
    return filtered;
}

void setMotorOutput(float output) {
    bool forward = (output >= 0.0f);
    int pwm = (int)fabsf(output);
    if (pwm < MOTOR_DEADBAND)
        pwm = 0;
    else if (pwm && pwm < MIN_PWM_KICK)
        pwm = MIN_PWM_KICK;
    pwm = constrain(pwm, 0, 255);
    digitalWrite(MOTOR_DIR_PIN, forward ? HIGH : LOW);
    analogWrite(MOTOR_PWM_PIN, pwm);
}
