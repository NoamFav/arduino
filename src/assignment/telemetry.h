// telemetry.h
#pragma once
#include "config.h"
#include "pid.h"

void sendStatusUpdate(float position, float error, float output, const PIDController &pid,
                      float target);
void updatePerformanceMetrics(PerformanceMetrics &m, float error);
void printHelp();
