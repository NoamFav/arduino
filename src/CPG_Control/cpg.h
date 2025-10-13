#pragma once
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

constexpr uint8_t I2C_ADDR = 0x40;
constexpr uint16_t SERVO_FREQ_HZ = 50;

constexpr double MIN_ANGLE = -90.0;
constexpr double MAX_ANGLE = +90.0;

constexpr size_t NUM_OSCILLATORS = 2;

extern const uint16_t SERVOMIN[NUM_OSCILLATORS];
extern const uint16_t SERVOMAX[NUM_OSCILLATORS];

extern double frequency, targetFrequency, rateOfFrequency;
extern double w, a, c;

extern bool telemetry_enabled;

struct Oscillator {
    double phase = 0.0;
    double amplitude = 8.0, targetAmplitude = 8.0;
    double offset = 0.0, targetOffset = 0.0; // centered at 0 for ±90
    double rateOfPhase = 0.0, rateOfAmplitude = 0.0, rateOfOffset = 0.0;
    double pos = 0.0;
    uint16_t angle_motor = 0;
    double phaseBias[NUM_OSCILLATORS]{};
    double coupling[NUM_OSCILLATORS]{};
};

extern Oscillator osc[NUM_OSCILLATORS];

extern double gait_speed_hz, gait_stride_deg, gait_bias_deg, gait_turn_rad;
void applyWalkGait();
void emitTelemetry(unsigned long now_ms);
void updateCPG(unsigned long dt);
void readInput();
