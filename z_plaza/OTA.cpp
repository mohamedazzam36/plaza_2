#include <WiFi.h>
#include <ArduinoOTA.h>
#include "OTA.h"
#include "Globals.h"

const char* ssid = "Z_Plaza";
const char* password = "ZPlaza123";

static bool otaInitialized = false;

void setupOTA() {
  xTaskCreatePinnedToCore(
    otaTask,
    "otaTask",
    4096,
    nullptr,
    1,
    nullptr,
    0
  );
}

void toggleOTA() {
  g_otaEnabled = !g_otaEnabled;
  
  if (g_otaEnabled) {
    WiFi.softAP(ssid, password);
    
    if (!otaInitialized) {
      ArduinoOTA.begin();
      otaInitialized = true;
    }
  
  } else {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
  }
}

void otaTask(void *pvParameters) {
  (void)pvParameters;
  while (true) {
    if (g_otaEnabled && otaInitialized) {
      ArduinoOTA.handle();
      vTaskDelay(pdMS_TO_TICKS(50));
    } else {
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}
