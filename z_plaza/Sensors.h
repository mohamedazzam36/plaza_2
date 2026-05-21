#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Config.h"
#include "Globals.h"
#include "I2Cdev.h"
#include "MPU6050.h"

extern MPU6050 mpu;

void initMpu();
void initUltrasonicPins();
int  readDistance(int trig, int echo);
int  readDistanceAvg(int trig, int echo);

#endif