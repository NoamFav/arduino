#include "io.h"

Adafruit_PWMServoDriver pwm(I2C_ADDR);

void initServos() {
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ_HZ);
    delay(4);
}

uint16_t angleToPwm(double angle, size_t idx) {
    long val = map(angle, 0, 180, SERVOMIN[idx], SERVOMAX[idx]);
    return constrain(val, SERVOMIN[idx], SERVOMAX[idx]);
}

void setServos() {
    for (size_t i = 0; i < NUM_OSCILLATORS; ++i) {
        osc[i].angle_motor = angleToPwm(osc[i].pos, i);
        pwm.setPWM(i, 0, osc[i].angle_motor);
    }
}
