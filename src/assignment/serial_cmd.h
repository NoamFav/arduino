// serial_cmd.h
#pragma once
#include "config.h"
#include "pid.h"

extern float target_position;
extern PerformanceMetrics metrics;
extern PIDController pid_controller;
extern bool telemetry_enabled;

void processSerialInput();
void processSerialCommand(const char *line);
