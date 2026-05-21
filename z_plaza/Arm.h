#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Bluepad32.h>
#include "Config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void initArm();

void processArmControl(ControllerPtr ctl);
void processShovelControl(ControllerPtr ctl);
void processSemiAutoControl(ControllerPtr ctl);

void toggleArmLock();
void toggleBaseLock();
void toggleShovelLock();
void resetArmLocks();

#endif
