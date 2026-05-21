#include "Globals.h"

// ================================================================
// ===              Global Variable Definitions                  ===
// ================================================================
volatile float         g_yaw           = 0.0f;
volatile float         g_gyroErrorZ    = 0.0f;
volatile int           g_frontDist     = 999;
volatile int           g_rightDist     = 999;
volatile int           g_leftDist      = 999;
volatile AutoFsmState  g_autoFsm       = ST_FORWARD;
volatile unsigned long g_fsmTimerMs    = 0;
volatile unsigned long g_forwardStartMs = 0;
volatile unsigned long g_lastMpuUs     = 0;
volatile OperationMode g_operationMode = MODE_WAIT_FOR_CONTROLLER;
volatile StraightMode  g_straightMode  = STRAIGHT_WITH_MPU; 

ControllerPtr g_activeController  = nullptr;
bool          g_prevL1R1Combo     = false;
bool          g_autoToggleLocked  = false;
bool          g_enableRgbRainbow  = true;
volatile bool g_otaEnabled        = false;
volatile ManualSubMode g_manualSubMode = SUBMODE_ARM;

portMUX_TYPE  g_mux = portMUX_INITIALIZER_UNLOCKED;

// FreeRTOS Task Handles
TaskHandle_t g_imuTaskHandle     = nullptr;
TaskHandle_t g_ultraTaskHandle   = nullptr;
TaskHandle_t g_controlTaskHandle = nullptr;
