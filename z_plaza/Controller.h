#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <Bluepad32.h>
#include "Globals.h"
#include "Motors.h"

// Bluepad32 callbacks
void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);

void processManualGamepad(ControllerPtr ctl);

void stopAllManualActuators();

#endif
