#include "Motors.h"

void initMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopCar();
}

int clampMotor(int v) {
  if (v >  MOTOR_PWM_MAX) return  MOTOR_PWM_MAX;
  if (v < -MOTOR_PWM_MAX) return -MOTOR_PWM_MAX;
  return v;
}

void leftMotor(int speed) {
  speed = clampMotor(speed);
  if (speed > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void rightMotor(int speed) {
  speed = clampMotor(speed);
  if (speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void turnRight(int speed) {
  rightMotor(-TURN_SIGN * speed);
  leftMotor(TURN_SIGN * speed);
}

void turnLeft(int speed) {
  rightMotor(TURN_SIGN * speed);
  leftMotor(-TURN_SIGN * speed);
}

void stopCar() {
  rightMotor(0);
  leftMotor(0);
}

void brakeFromForward() {
  rightMotor(-BRAKE_POWER);
  leftMotor(-BRAKE_POWER);
}