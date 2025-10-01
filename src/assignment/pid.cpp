// pid.cpp
#include "pid.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

float executePIDStep(PIDController &c, float error) {
    const float p = c.kp * error;
    const float d = c.kd * (error - c.previous_error) / c.sample_time;

    float proposed = c.integral_sum;
    if (fabsf(error) > INTEGRAL_DEADBAND)
        proposed += error;

    float tentative = p + c.ki * c.sample_time * proposed + d;
    bool sat_hi = (tentative > c.output_max) && (error > 0);
    bool sat_lo = (tentative < c.output_min) && (error < 0);
    if (!sat_hi && !sat_lo)
        c.integral_sum = proposed;

    float out = p + c.ki * c.sample_time * c.integral_sum + d;
    out = constrain(out, c.output_min, c.output_max);
    c.previous_error = error;
    return out;
}
