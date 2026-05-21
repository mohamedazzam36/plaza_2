#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Bluepad32.h>
#include "Config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ================================================================
// ===                      Enumerations                         ===
// ================================================================
enum AutoFsmState {
  ST_FORWARD,
  ST_BRAKING,
  ST_PAUSE_BEFORE_SCAN,
  ST_SCAN_SIDES,
  ST_TURNING
};

enum OperationMode : uint8_t {
  MODE_WAIT_FOR_CONTROLLER = 0,
  MODE_AUTO   = 1,
  MODE_MANUAL = 2
};

enum ManualSubMode : uint8_t {
  SUBMODE_ARM       = 0,
  SUBMODE_SHOVEL    = 1,
  SUBMODE_SEMI_AUTO = 2
};

enum StraightMode : uint8_t {
  STRAIGHT_WITH_WALLS = 0,
  STRAIGHT_WITH_MPU   = 1,
  STRAIGHT_NO_PID     = 2
};

// ================================================================
// ===                   Shared Variables                         ===
// ================================================================
extern volatile float         g_yaw;
extern volatile float         g_gyroErrorZ;
extern volatile int           g_frontDist;
extern volatile int           g_rightDist;
extern volatile int           g_leftDist;
extern volatile AutoFsmState  g_autoFsm;
extern volatile unsigned long g_fsmTimerMs;
extern volatile unsigned long g_forwardStartMs;
extern volatile unsigned long g_lastMpuUs;
extern volatile OperationMode g_operationMode;
extern volatile StraightMode  g_straightMode;

extern ControllerPtr g_activeController;
extern bool          g_prevL1R1Combo;
extern bool          g_autoToggleLocked;
extern bool          g_enableRgbRainbow;
extern volatile bool g_otaEnabled;
extern volatile ManualSubMode g_manualSubMode;


extern portMUX_TYPE  g_mux;

// FreeRTOS Task Handles (for suspend/resume)
extern TaskHandle_t g_imuTaskHandle;
extern TaskHandle_t g_ultraTaskHandle;
extern TaskHandle_t g_controlTaskHandle;

#endif
