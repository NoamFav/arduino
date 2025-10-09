#pragma once
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

constexpr uint8_t I2C_ADDR = 0x40;
constexpr uint16_t SERVO_FREQ_HZ = 50;
constexpr size_t NUM_OSCILLATORS = 3;

extern const uint16_t SERVOMIN[NUM_OSCILLATORS];
extern const uint16_t SERVOMAX[NUM_OSCILLATORS];

extern double frequency, targetFrequency, rateOfFrequency;
extern double w, a, c;

struct Oscillator {
    double phase, amplitude, targetAmplitude;
    double offset, targetOffset;
    double rateOfPhase, rateOfAmplitude, rateOfOffset;
    double pos;
    uint16_t angle_motor;
    double phaseBias[NUM_OSCILLATORS];
    double coupling[NUM_OSCILLATORS];
};

extern Oscillator osc[NUM_OSCILLATORS];

void updateCPG(unsigned long dt);
