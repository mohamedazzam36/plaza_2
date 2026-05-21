#include "Sensors.h"

MPU6050 mpu(MPU_ADDR);

void initMpu() {
  mpu.initialize();
  // ---- Auto-calibrate gyro Z error ----
  long sumGZ = 0;
  for (int i = 0; i < 500; i++) {
    sumGZ += mpu.getRotationZ();
    delay(2);
  }
  g_gyroErrorZ = (float)sumGZ / 500.0f;

  portENTER_CRITICAL(&g_mux);
  g_lastMpuUs = micros();
  g_yaw       = 0.0f;
  portEXIT_CRITICAL(&g_mux);
}

void initUltrasonicPins() {
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG,  OUTPUT);
  pinMode(LEFT_ECHO,  INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
}

int readDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 30000);
  if (d == 0) return 999;
  return (int)(d * 0.034f) / 2;
}

int readDistanceAvg(int trig, int echo) {
  int sum = 0;
  for (int i = 0; i < US_SIDE_SAMPLES; i++) {
    sum += readDistance(trig, echo);
    vTaskDelay(pdMS_TO_TICKS(6));
  }
  return sum / US_SIDE_SAMPLES;
}