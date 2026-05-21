#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <Arduino.h>
#include <PID_v1.h>
#include "Config.h"
#include "Globals.h"
#include "Motors.h"
#include "Sensors.h"

// PID variables
extern double yawInput, yawOutput, yawSetpoint;
extern double wallInput, wallOutput, wallSetpoint;

// FreeRTOS Tasks
void imuTask(void *pvParameters);
void ultrasonicTask(void *pvParameters);
void controlTask(void *pvParameters);


int getDynamicStopDistance(unsigned long nowMs, unsigned long forwardStartMs);

#endif
