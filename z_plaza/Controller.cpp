#include "Controller.h"
#include "Arm.h"
#include "OTA.h"
#include "Sensors.h"
#include <math.h>

// ================================================================
// ===                  Helper: Mode Switching                   ===
// ================================================================

// --- Manual sub-mode state ---
static ManualSubMode manualSubMode     = SUBMODE_ARM;
static ManualSubMode prevNormalSubMode = SUBMODE_ARM;  // remembers ARM or SHOVEL before semi-auto
static bool    prevPsBtn              = false;
static bool    prevR3Btn              = false;
static bool    prevDpadUp             = false;
static bool    prevShareOptionsCombo  = false;

static void switchToManualMode() {
  portENTER_CRITICAL(&g_mux);
  g_operationMode = MODE_MANUAL;
  portEXIT_CRITICAL(&g_mux);

  stopAllManualActuators();

  manualSubMode = SUBMODE_ARM;
  prevPsBtn     = false;
  prevR3Btn     = false;
  resetArmLocks();

  if (g_imuTaskHandle)     vTaskSuspend(g_imuTaskHandle);
  if (g_controlTaskHandle) vTaskSuspend(g_controlTaskHandle);
  if (g_ultraTaskHandle)   vTaskSuspend(g_ultraTaskHandle);
}

static void switchToAutoMode() {
  stopAllManualActuators();

  // ---- Re-calibrate MPU gyro Z before entering AUTO ----
  long sumGZ = 0;
  for (int i = 0; i < 300; i++) {
    sumGZ += mpu.getRotationZ();
    delay(2);
  }
  g_gyroErrorZ = (float)sumGZ / 300.0f;
  // -------------------------------------------------------

  portENTER_CRITICAL(&g_mux);
  g_lastMpuUs      = micros();
  g_yaw            = 0.0f;
  g_autoFsm        = ST_FORWARD;
  g_fsmTimerMs     = 0;
  g_forwardStartMs = millis();
  g_frontDist      = 999;
  g_rightDist      = 999;
  g_leftDist       = 999;
  g_operationMode  = MODE_AUTO;
  portEXIT_CRITICAL(&g_mux);

  if (g_imuTaskHandle)     vTaskResume(g_imuTaskHandle);
  if (g_controlTaskHandle) vTaskResume(g_controlTaskHandle);
  if (g_ultraTaskHandle)   vTaskResume(g_ultraTaskHandle);
}

// ================================================================
// ===                  Bluepad32 Callbacks                      ===
// ================================================================
void onConnectedController(ControllerPtr ctl) {
  if (g_activeController == nullptr) {
    g_activeController = ctl;
    BP32.enableNewBluetoothConnections(false);
  } else {
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (g_activeController == ctl) {
    g_activeController = nullptr;
    g_prevL1R1Combo = false;
    stopAllManualActuators();

    if (g_imuTaskHandle)     vTaskSuspend(g_imuTaskHandle);
    if (g_controlTaskHandle) vTaskSuspend(g_controlTaskHandle);
    if (g_ultraTaskHandle)   vTaskSuspend(g_ultraTaskHandle);

    portENTER_CRITICAL(&g_mux);
    g_operationMode = MODE_WAIT_FOR_CONTROLLER;
    portEXIT_CRITICAL(&g_mux);
    BP32.enableNewBluetoothConnections(true);
  }
}

// ================================================================
// ===                    Manual Gamepad                          ===
// ================================================================
void stopAllManualActuators() {
  stopCar();
}

void processManualGamepad(ControllerPtr ctl) {
  bool combo = ctl->l1() && ctl->r1();
  if (combo && !g_prevL1R1Combo) {
    OperationMode modeLocal;
    portENTER_CRITICAL(&g_mux);
    modeLocal = g_operationMode;
    portEXIT_CRITICAL(&g_mux);

    if (!g_autoToggleLocked) {
      if (modeLocal == MODE_AUTO) {
        switchToManualMode();
        g_autoToggleLocked = true;
      } else {
        switchToAutoMode();
      }
    }
  }
  g_prevL1R1Combo = combo;

  // ============================================================
  // --- OTA Mode Combo ---
  // ============================================================
  bool shareOptionsCombo = (ctl->miscButtons() & 0x06) == 0x06;
  if (shareOptionsCombo && !prevShareOptionsCombo) {
    toggleOTA();
  }
  prevShareOptionsCombo = shareOptionsCombo;


  OperationMode modeLocal;
  portENTER_CRITICAL(&g_mux);
  modeLocal = g_operationMode;
  portEXIT_CRITICAL(&g_mux);
  if (modeLocal != MODE_MANUAL) return;

  
  // ============================================================
  // --- Sub-mode Toggle (PS button: ARM <-> SHOVEL) ---
  // ============================================================
  bool psBtn = ctl->miscButtons() & 0x01;
  if (psBtn && !prevPsBtn) {
    if (manualSubMode == SUBMODE_ARM || manualSubMode == SUBMODE_SEMI_AUTO) {
      manualSubMode = SUBMODE_SHOVEL;
    } else {
      manualSubMode = SUBMODE_ARM;
    }
    if (manualSubMode != SUBMODE_SEMI_AUTO) prevNormalSubMode = manualSubMode;
    g_manualSubMode = manualSubMode;
  }
  prevPsBtn = psBtn;

  // ============================================================
  // --- Semi-Auto Toggle (D-pad Up: normal <-> SEMI_AUTO) ---
  // ============================================================
  uint8_t dp     = ctl->dpad();
  bool dpadUp    = dp & 0x08;   // DPAD_UP
  if (dpadUp && !prevDpadUp) {
    if (manualSubMode != SUBMODE_SEMI_AUTO) {
      prevNormalSubMode = manualSubMode;   // remember ARM or SHOVEL
      manualSubMode = SUBMODE_SEMI_AUTO;
    } else {
      manualSubMode = prevNormalSubMode;   // restore previous
    }
    g_manualSubMode = manualSubMode;
  }
  prevDpadUp = dpadUp;

  // ============================================================
  // --- Lock Toggle ---
  // ============================================================
  uint16_t btns = ctl->buttons();
  bool r3Btn = btns & 0x0200;   // BUTTON_THUMB_R = R3 click
  if (r3Btn && !prevR3Btn) {
    if (manualSubMode == SUBMODE_SHOVEL) {
      toggleShovelLock();
    } else {
      toggleBaseLock();   // ARM & SEMI_AUTO: lock base servo only
    }
  }
  prevR3Btn = r3Btn;

  // ============================================================
  // --- Manual Driving ---
  // ============================================================
  float throttle = (ctl->throttle() - ctl->brake()) / 1023.0f;

  int rawAxisX = ctl->axisX();
  float steer = 0.0f;
  if (abs(rawAxisX) > DRIVE_STEER_DEADZONE) {
    steer = (float)(rawAxisX - (rawAxisX > 0 ? DRIVE_STEER_DEADZONE : -DRIVE_STEER_DEADZONE))
            / (512.0f - DRIVE_STEER_DEADZONE) * 1.0f;
  }

  float leftF  = throttle + steer;
  float rightF = throttle - steer;
  float m = fmax(fabs(leftF), fabs(rightF));
  if (m > 1.0f) { leftF /= m; rightF /= m; }

  leftMotor((int)(leftF * 255));
  rightMotor((int)(rightF * 255));
  // ============================================================
  // --- Sub-mode Processing ---
  // ============================================================
  if (manualSubMode == SUBMODE_ARM) {
    processArmControl(ctl);
  } else if (manualSubMode == SUBMODE_SHOVEL) {
    processShovelControl(ctl);
  } else {   // SUBMODE_SEMI_AUTO
    processSemiAutoControl(ctl);
  }
}

