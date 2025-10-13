#include "io.h"

Adafruit_PWMServoDriver pwm(I2C_ADDR);

static inline uint16_t mapAngleToPwm(double angleDeg, size_t idx) {
    if (idx >= NUM_OSCILLATORS)
        idx = NUM_OSCILLATORS - 1;
    if (angleDeg < MIN_ANGLE)
        angleDeg = MIN_ANGLE;
    if (angleDeg > MAX_ANGLE)
        angleDeg = MAX_ANGLE;

    const double in0 = MIN_ANGLE, in1 = MAX_ANGLE;
    const double out0 = SERVOMIN[idx], out1 = SERVOMAX[idx];
    const double t = (angleDeg - in0) / (in1 - in0);
    const double v = out0 + t * (out1 - out0);
    const long vv = lround(v);
    return (uint16_t)constrain(vv, (long)SERVOMIN[idx], (long)SERVOMAX[idx]);
}

void initServos() {
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ_HZ);
    delay(4);
}

void setServos() {
    for (size_t i = 0; i < NUM_OSCILLATORS; ++i) {
        osc[i].angle_motor = mapAngleToPwm(osc[i].pos, i);
        pwm.setPWM((uint8_t)i, 0, osc[i].angle_motor);
    }
}
