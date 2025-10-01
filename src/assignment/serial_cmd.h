// serial_cmd.h
#pragma once
#include "config.h"
#include "pid.h"

extern float target_position;        // defined in .ino (single owner)
extern PerformanceMetrics metrics;   // defined in .ino
extern PIDController pid_controller; // defined in .ino

void processSerialInput();
void processSerialCommand(const char *line);
