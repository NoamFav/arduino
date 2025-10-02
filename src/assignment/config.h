#pragma once
#include <Arduino.h>

#define MOTOR_PWM_PIN 9
#define MOTOR_DIR_PIN 20
#define POTENTIOMETER_PIN A0

static const float SAMPLE_TIME_S = 0.010f;
static const uint32_t SAMPLE_TIME_MS = (uint32_t)(SAMPLE_TIME_S * 1000);

static const int MOTOR_OUTPUT_MIN = -255;
static const int MOTOR_OUTPUT_MAX = 255;
static const int MIN_PWM_KICK = 10;
static const int MOTOR_DEADBAND = 2;
static const float INTEGRAL_DEADBAND = 2.0f;
static const float SMOOTHING_ALPHA = 0.9f;

static const float POT_RAW_MIN = 200.0f;
static const float POT_RAW_MAX = 800.0f;
static const float POT_RANGE = 1023.0f;

enum OutputFormat : uint8_t {
    OUTPUT_MODE_PLOTTER = 0,
    OUTPUT_MODE_VERBOSE = 1,
    OUTPUT_MODE_JSON = 2
};

static const uint8_t OUTPUT_FORMAT = 0;

struct PerformanceMetrics {
    uint32_t loop_count = 0, last_report_time = 0;
    float max_error = 0.0f, avg_error = 0.0f, error_sum = 0.0f;
    static const uint32_t REPORT_INTERVAL_MS = 5000;
};
