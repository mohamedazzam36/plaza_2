#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "Config.h"

void initMotors();

void rightMotor(int speed);
void leftMotor(int speed);
void stopCar();
void brakeFromForward();

void turnRight(int speed);
void turnLeft(int speed);

int clampMotor(int v);

#endif