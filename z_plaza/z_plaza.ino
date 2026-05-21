// ================================================================
// ===                    Master Robot Firmware                  ===
// ===          ESP32 + Bluepad32 + MPU6050 + PID + FreeRTOS    ===
// ================================================================

#include "Wire.h"
#include <Bluepad32.h>

#include "Config.h"
#include "Globals.h"
#include "Motors.h"
#include "Sensors.h"
#include "Controller.h"
#include "AutoPilot.h"
#include "RGBLight.h"
#include "Arm.h"
#include "OTA.h"

// ================================================================
// ===                         Setup                             ===
// ================================================================
void setup() {

  // I2C
  Wire.begin();
  Wire.setClock(400000);

  // Hardware init
  initMotors();
  initUltrasonicPins();
  initArm();
  initMpu();
  setupOTA();

  yawSetpoint = 0.0;
  stopAllManualActuators();

  portENTER_CRITICAL(&g_mux);
  g_operationMode  = MODE_WAIT_FOR_CONTROLLER;
  g_forwardStartMs = 0;
  g_autoFsm        = ST_FORWARD;
  portEXIT_CRITICAL(&g_mux);

  // Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);
  BP32.enableNewBluetoothConnections(true);

  // FreeRTOS Tasks — auto tasks self-suspend on entry, resumed by L1+R1
  xTaskCreatePinnedToCore(imuTask,        "imuTask",     4096, nullptr, 3, &g_imuTaskHandle,     1);
  xTaskCreatePinnedToCore(controlTask,    "controlTask", 6144, nullptr, 2, &g_controlTaskHandle,  1);
  xTaskCreatePinnedToCore(ultrasonicTask, "ultraTask",   4096, nullptr, 1, &g_ultraTaskHandle,    0);
  xTaskCreatePinnedToCore(rgbTask,        "rgbTask",     2048, nullptr, 1, nullptr,               0);
}

// ================================================================
// ===                          Loop                             ===
// ================================================================
void loop() {
  BP32.update();
  if (g_activeController && g_activeController->isConnected() && g_activeController->isGamepad()) {
    processManualGamepad(g_activeController);
  }
  vTaskDelay(pdMS_TO_TICKS(20));
}