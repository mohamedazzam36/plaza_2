#include "RGBLight.h"

// ================================================================
// ===                  HSV → RGB Conversion                     ===
// ================================================================
static void hsvToRgb(uint8_t h, uint8_t s, uint8_t v,
                     uint8_t &r, uint8_t &g, uint8_t &b) {
  uint8_t region    = h / 43;
  uint8_t remainder = (h - (region * 43)) * 6;

  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region) {
    case 0:  r = v; g = t; b = p; break;
    case 1:  r = q; g = v; b = p; break;
    case 2:  r = p; g = v; b = t; break;
    case 3:  r = p; g = q; b = v; break;
    case 4:  r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
  }
}

// ================================================================
// ===                    RGB FreeRTOS Task                       ===
// ================================================================
void rgbTask(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWake = xTaskGetTickCount();
  uint8_t hue = 0;
  uint8_t policeCounter = 0;
  bool isRed = true;
  bool ledWasOn = true;

  while (true) {
    ControllerPtr ctl = nullptr;
    OperationMode modeLocal;

    portENTER_CRITICAL(&g_mux);
    if (g_activeController && g_activeController->isConnected() && g_activeController->isGamepad()) {
      ctl = g_activeController;
    }
    modeLocal = g_operationMode;
    portEXIT_CRITICAL(&g_mux);

    if (ctl) {
      if (g_otaEnabled) {
        ctl->setColorLED(255, 0, 255);
        ledWasOn = true;
      } else if (g_enableRgbRainbow) {
        ledWasOn = true;

        if (modeLocal == MODE_AUTO) {
          policeCounter++;
          if (policeCounter >= 8) {
            isRed = !isRed;
            policeCounter = 0;
          }
          if (isRed) {
            ctl->setColorLED(255, 0, 0);
          } else {
            ctl->setColorLED(0, 0, 255);
          }
        } else if (modeLocal == MODE_MANUAL) {
          ManualSubMode subMode = g_manualSubMode;
          if (subMode == SUBMODE_SEMI_AUTO) {
            // Semi-Auto: solid green
            ctl->setColorLED(0, 255, 0);
          } else if (subMode == SUBMODE_SHOVEL) {
            // Shovel: light off
            ctl->setColorLED(0, 0, 0);
          } else {
            // ARM: rainbow
            uint8_t r, g, b;
            hsvToRgb(hue, 255, 150, r, g, b);
            ctl->setColorLED(r, g, b);
            hue++;
          }
        }
      } else if (ledWasOn) {
        ctl->setColorLED(0, 0, 255);
        ledWasOn = false;
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(30));
  }
}
