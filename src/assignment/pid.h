// pid.h
#pragma once
#include "config.h"

struct PIDController {
    float kp = 0.8f, ki = 0.25f, kd = 0.02f;
    float sample_time = SAMPLE_TIME_S;
    float integral_sum = 0.0f;
    float previous_error = 0.0f;
    float output_min = MOTOR_OUTPUT_MIN, output_max = MOTOR_OUTPUT_MAX;
};

float executePIDStep(PIDController &c, float error);
