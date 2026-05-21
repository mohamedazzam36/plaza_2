#include "AutoPilot.h"
#include <math.h>

// ================================================================
// ===                    PID Objects                            ===
// ================================================================
// MPU straight-line PID
double yawInput = 0.0, yawOutput = 0.0, yawSetpoint = 0.0;
PID straightPid(&yawInput, &yawOutput, &yawSetpoint, KP_STRAIGHT, KI_STRAIGHT, KD_STRAIGHT, DIRECT);

double wallInput = 0.0, wallOutput = 0.0, wallSetpoint = 0.0;
PID wallPid(&wallInput, &wallOutput, &wallSetpoint, KP_WALL, KI_WALL, KD_WALL, DIRECT);

PID turnPid(&yawInput, &yawOutput, &yawSetpoint, KP_TURN, KI_TURN, KD_TURN, DIRECT);

// ================================================================
// ===                     Helpers                               ===
// ================================================================
int getDynamicStopDistance(unsigned long nowMs, unsigned long forwardStartMs) {
  if (MAX_STOP_DIST <= MIN_STOP_DIST) return MIN_STOP_DIST;
  if (forwardStartMs == 0) return MIN_STOP_DIST;

  unsigned long elapsedMs = nowMs - forwardStartMs;
  if (elapsedMs >= FORWARD_RAMP_TIME_MS) return MAX_STOP_DIST;

  float ratio = (float)elapsedMs / (float)FORWARD_RAMP_TIME_MS;
  int dynamicStop = MIN_STOP_DIST + (int)((MAX_STOP_DIST - MIN_STOP_DIST) * ratio);
  return constrain(dynamicStop, MIN_STOP_DIST, MAX_STOP_DIST);
}

// ================================================================
// ===                    IMU Task                               ===
// ================================================================
void imuTask(void *pvParameters) {
  (void)pvParameters;

  vTaskSuspend(NULL);

  TickType_t lastWake = xTaskGetTickCount();

  while (true) {

    unsigned long nowUs = micros();
    float yawLocal;
    unsigned long prevUs;

    portENTER_CRITICAL(&g_mux);
    prevUs      = g_lastMpuUs;
    g_lastMpuUs = nowUs;
    yawLocal    = g_yaw;
    portEXIT_CRITICAL(&g_mux);

    float dt = (nowUs - prevUs) / 1000000.0f;
    if (prevUs == 0 || dt <= 0.0f) dt = 0.01f;
    if (dt > 0.05f) dt = 0.05f;

    int16_t rawZ = mpu.getRotationZ();
    float gZdeg  = (rawZ - g_gyroErrorZ) / 131.0f;
    yawLocal    += gZdeg * dt;

    portENTER_CRITICAL(&g_mux);
    g_yaw = yawLocal;
    portEXIT_CRITICAL(&g_mux);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_TASK_MS));
  }
}

// ================================================================
// ===                 Ultrasonic Task                           ===
// ================================================================
void ultrasonicTask(void *pvParameters) {
  (void)pvParameters;

  vTaskSuspend(NULL);

  TickType_t lastWake = xTaskGetTickCount();

  while (true) {

    AutoFsmState  stateLocal;
    StraightMode  straightLocal;
    portENTER_CRITICAL(&g_mux);
    stateLocal    = g_autoFsm;
    straightLocal = g_straightMode;
    portEXIT_CRITICAL(&g_mux);

    int front = readDistanceAvg(FRONT_TRIG, FRONT_ECHO);
    int right = g_rightDist;
    int left  = g_leftDist;

    bool needSides = (stateLocal == ST_SCAN_SIDES) ||
                     (stateLocal == ST_FORWARD && straightLocal == STRAIGHT_WITH_WALLS);

    if (needSides) {
      right = readDistanceAvg(RIGHT_TRIG, RIGHT_ECHO);
      left  = readDistanceAvg(LEFT_TRIG,  LEFT_ECHO);
    }

    portENTER_CRITICAL(&g_mux);
    g_frontDist = front;
    g_rightDist = right;
    g_leftDist  = left;
    portEXIT_CRITICAL(&g_mux);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(ULTRASONIC_TASK_MS));
  }
}

// ================================================================
// ===                   Control FSM Task                        ===
// ================================================================
void controlTask(void *pvParameters) {
  (void)pvParameters;

  straightPid.SetMode(AUTOMATIC);
  straightPid.SetOutputLimits(-LIMIT_STRAIGHT, LIMIT_STRAIGHT);
  straightPid.SetSampleTime(CONTROL_TASK_MS);

  wallPid.SetMode(AUTOMATIC);
  wallPid.SetOutputLimits(-LIMIT_WALL, LIMIT_WALL);
  wallPid.SetSampleTime(CONTROL_TASK_MS);

  turnPid.SetMode(AUTOMATIC);
  turnPid.SetControllerDirection(REVERSE);
  turnPid.SetOutputLimits(-LIMIT_TURN, LIMIT_TURN);
  turnPid.SetSampleTime(CONTROL_TASK_MS);

  vTaskSuspend(NULL);

  TickType_t lastWake = xTaskGetTickCount();

  while (true) {

    float yawLocal;
    int frontLocal, rightLocal, leftLocal;
    AutoFsmState stateLocal;
    unsigned long fsmTimerLocal;
    unsigned long forwardStartLocal;

    portENTER_CRITICAL(&g_mux);
    yawLocal          = g_yaw;
    frontLocal        = g_frontDist;
    rightLocal        = g_rightDist;
    leftLocal         = g_leftDist;
    stateLocal        = g_autoFsm;
    fsmTimerLocal     = g_fsmTimerMs;
    forwardStartLocal = g_forwardStartMs;
    portEXIT_CRITICAL(&g_mux);

    unsigned long nowMs = millis();
    yawInput = yawLocal;

    switch (stateLocal) {
      case ST_FORWARD: {
        int leftSpeed  = DEFAULT_CAR_SPEED;
        int rightSpeed = DEFAULT_CAR_SPEED;
        int stopDistDynamic = getDynamicStopDistance(nowMs, forwardStartLocal);

        StraightMode modeLocal;
        portENTER_CRITICAL(&g_mux);
        modeLocal = g_straightMode;
        portEXIT_CRITICAL(&g_mux);

        if (modeLocal == STRAIGHT_WITH_MPU) {
          yawInput = yawLocal;
          straightPid.Compute();
          if (yawOutput < 0) {
            leftSpeed  = constrain(DEFAULT_CAR_SPEED - (int)yawOutput,         0, MOTOR_PWM_MAX);
            rightSpeed = constrain(DEFAULT_CAR_SPEED + (int)(yawOutput * 0.7), 0, MOTOR_PWM_MAX);
          } else {
            rightSpeed = constrain(DEFAULT_CAR_SPEED + (int)yawOutput,         0, MOTOR_PWM_MAX);
            leftSpeed  = constrain(DEFAULT_CAR_SPEED - (int)(yawOutput * 0.7), 0, MOTOR_PWM_MAX);
          }

        } else if (modeLocal == STRAIGHT_WITH_WALLS) {
          bool wallReady = (forwardStartLocal != 0) &&
                           ((nowMs - forwardStartLocal) >= WALL_MODE_ACTIVATE_MS);
          
          bool wallValid = (leftLocal <= WALL_CRITICAL_DIST_CM) || 
                           (rightLocal <= WALL_CRITICAL_DIST_CM);

          if (wallReady && wallValid) {
            wallInput    = (double)(leftLocal - rightLocal);
            wallSetpoint = 0.0;
            wallPid.Compute();
            leftSpeed  = constrain(DEFAULT_CAR_SPEED + (int)wallOutput, 0, MOTOR_PWM_MAX);
            rightSpeed = constrain(DEFAULT_CAR_SPEED - (int)wallOutput, 0, MOTOR_PWM_MAX);
          }
          // else: drive straight at DEFAULT_CAR_SPEED with no correction

        }


        rightMotor(rightSpeed);
        leftMotor(leftSpeed);

        if (frontLocal <= stopDistDynamic) {
          portENTER_CRITICAL(&g_mux);
          g_autoFsm    = ST_BRAKING;
          g_fsmTimerMs = nowMs;
          portEXIT_CRITICAL(&g_mux);
        }
        break;
      }

      case ST_BRAKING:
        brakeFromForward();
        if (nowMs - fsmTimerLocal >= BRAKE_DURATION_MS) {
          stopCar();
          portENTER_CRITICAL(&g_mux);
          g_autoFsm    = ST_PAUSE_BEFORE_SCAN;
          g_fsmTimerMs = nowMs;
          portEXIT_CRITICAL(&g_mux);
        }
        break;

      case ST_PAUSE_BEFORE_SCAN:
        stopCar();
        if (nowMs - fsmTimerLocal >= PAUSE_BEFORE_SCAN_MS) {
          portENTER_CRITICAL(&g_mux);
          g_autoFsm = ST_SCAN_SIDES;
          portEXIT_CRITICAL(&g_mux);
        }
        break;

      case ST_SCAN_SIDES:
        stopCar();
        rightLocal = readDistanceAvg(RIGHT_TRIG, RIGHT_ECHO);
        leftLocal  = readDistanceAvg(LEFT_TRIG,  LEFT_ECHO);

        portENTER_CRITICAL(&g_mux);
        g_rightDist = rightLocal;
        g_leftDist  = leftLocal;
        portEXIT_CRITICAL(&g_mux);

        if (leftLocal > rightLocal) {
          yawSetpoint = yawLocal + (float)TURN_SET_POINT;
        } else {
          yawSetpoint = yawLocal - (float)TURN_SET_POINT;
        }
        portENTER_CRITICAL(&g_mux);
        g_autoFsm = ST_TURNING;
        portEXIT_CRITICAL(&g_mux);
        break;

      case ST_TURNING: {
        turnPid.Compute();
        int speed = abs((int)yawOutput);
        if (speed < MINIMUM_TURN_SPEED) speed = MINIMUM_TURN_SPEED;

        float turnErr = yawSetpoint - yawLocal;
        if (turnErr > 0) {
          turnLeft(speed);
        } else {
          turnRight(speed);
        }

        if (fabs((double)yawLocal - yawSetpoint) <= (double)TURN_ANGLE_TOLERANCE) {
          stopCar();
          yawSetpoint = yawLocal;
          portENTER_CRITICAL(&g_mux);
          g_autoFsm      = ST_FORWARD;
          g_forwardStartMs = nowMs;
          portEXIT_CRITICAL(&g_mux);
        }
        break;
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_TASK_MS));
  }
}
