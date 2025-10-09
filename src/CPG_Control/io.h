#pragma once
#include "cpg.h"
#include <Arduino.h>

extern Adafruit_PWMServoDriver pwm;

void initServos();
void setServos();
uint16_t angleToPwm(double angle, size_t idx);
