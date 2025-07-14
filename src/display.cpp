#include "display.h" 
#include "main.h"   
#include "gbw.h"
#include "definitions.h"

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>             // Arduino SPI library
#include <math.h>

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>

#ifdef RT_DRIVE 
#include "comm_rt.h"
#include "motorcontrol_rt.h"
#endif 
#ifdef JMC_DRIVE
#include "comm_jmc.h"
#include "motorcontrol_jmc.h"
#endif

#define COLOR_NAVY 0x0010
#define COLOR_GREEN 0x4fe9
#define COLOR_RED 0xe803
#define COLOR_YELLOW 0xe7e0

const int SCREEN_WIDTH = 280;
const int SCREEN_HEIGHT = 240;
Adafruit_ST7789 *tft = NULL;

#define SCREEN_IDLE      0
#define SCREEN_GBW_IDLE  1
#define SCREEN_GRINDING  2
#define SCREEN_PURGING   3
#define SCREEN_GRINDING_GBW 5
#define SCREEN_CALIBRATING  6
#define SCREEN_MENU         4

int initializedScreenId = -1;
bool dimmed = false;
bool disp_updateRequired = true;


void display_init() { 
  SPIClass *spi = new SPIClass(HSPI);
  spi->begin(DISP_SCL, -1, DISP_SDA, DISP_CS);
  tft = new Adafruit_ST7789(spi, DISP_CS, DISP_DC, DISP_RST);
  // 80MHz should work, but you may need lower speeds
  tft->setSPISpeed(80000000);
  // this will vary depending on your display
  tft->init(SCREEN_HEIGHT, SCREEN_WIDTH, SPI_MODE0);
  tft->setRotation(1); // Landscape

  // Set backlight pin as output and turn on backlight
  //pinMode(DISP_BL, OUTPUT);
  ledcSetup(1,4000,8);
  ledcAttachPin(DISP_BL, 1);
  ledcWrite(1, 0);
  update_display();
}

void setBrightness() { 
  static uint8_t lastBrightness;
  uint8_t newBrightness;
  static int dimTime = 15; 

  if(lastActivity + dimTime * 60000 < millis()) { 
    newBrightness = 1;
    dimmed = true;
  } else {
      newBrightness = Menu1[DISP_BRIGHTNESS].value;
      dimmed = false;
  }

  if(lastBrightness != newBrightness) { 
    ledcWrite(1, newBrightness);
    lastBrightness = newBrightness;
  }
}

void update_display() { 
  static unsigned long lastUpdate;
  if(newData == true) disp_updateRequired = true;

  if(disp_updateRequired != true) return; //This is mostly functional for testing.
  if(lastUpdate + 1000/50 > millis()) return;

  lastUpdate = millis();
  disp_updateRequired = false;

  setBrightness();

    if(state == IDLE) { 
      drawIdleScreen();
    }

    if(state == IDLE_GBW) { 
      drawGbWIdleScreen();
    }

    if(state == GRINDING) { 
      drawGrindingOrPurgingScreen();
    }

    if(state == PURGING) { 
      drawGrindingOrPurgingScreen();
    }

    if(state == GRINDING_GBW) { 
      drawGrindingGbwScreen();
    }

    if(state == MENU1) { 
      enterMenu == true ? drawMenuValueScreen(1, menu1Selected) : drawMenuScreen(1, NUM_MENU1_ITEMS, menu1Selected);
    }

    if(state == MENU2) { 
      enterMenu == true ? drawMenuValueScreen(2, menu2Selected) : drawMenuScreen(2, NUM_MENU2_ITEMS, menu2Selected);
    }

    if(state == MENU3) { 
      enterMenu == true ? drawMenuValueScreen(3, menu3Selected) : drawMenuScreen(3, NUM_MENU3_ITEMS, menu3Selected);
    }

    if(state == CALIBRATING) { 
      drawCalibratingScreen();
    }
}

void display_off() {
    if (tft) {
        tft->fillScreen(ST77XX_BLACK);
        tft->enableDisplay(false); // turn off display output (if supported)
    }
    ledcWrite(1,0);
}

const char* getCommStatusText(SYSTEM_STATUS status) {
  switch (status) {
    case DISCONNECTED: return "Disconnected";
    case ERROR:        return "ERROR";
    case CONNECTED:    return "Connected";
    case READY:        return "Ready";
    default:           return "UNKNOWN";
  }
}

uint16_t getCommStatusColor(SYSTEM_STATUS status) {
  switch (status) {
    case CONNECTED: return COLOR_GREEN;
    case READY: return COLOR_GREEN;
    case DISCONNECTED: return COLOR_RED;
    case ERROR: return COLOR_YELLOW;
    default: return ST77XX_WHITE;
  }
}

const char* getScaleStatusText(SCALE_STATUS scaleStatus) {
  switch (scaleStatus) {
    case SCALE_DISCONNECTED: return "Disconnected";
    case SCALE_CONNECTED:        return "Connected";
    case SCALE_CONNECTING:    return "Connecting..";
    case SCALE_LOST:        return "Lost..";
    default:           return "UNKNOWN";
  }
}

uint16_t getScaleStatusColor(SCALE_STATUS status) {
  switch (status) {
    case SCALE_CONNECTED: return COLOR_GREEN;
    case SCALE_DISCONNECTED: return COLOR_RED;
    case SCALE_CONNECTING: return COLOR_YELLOW;
    case SCALE_LOST: return COLOR_YELLOW;
    default: return ST77XX_WHITE;
  }
}

void drawIdleScreen() {
  static int prevSetRPM = -1;
  static SYSTEM_STATUS prevCommStatus = INVALID_STATUS;
  

  int16_t x1, y1;
  uint16_t w, h;

  if (initializedScreenId != SCREEN_IDLE) {
    tft->fillScreen(ST77XX_BLACK);

    int lineY = SCREEN_HEIGHT * 0.7;
    int lineW = SCREEN_WIDTH * 0.8;
    int lineX = (SCREEN_WIDTH - lineW) / 2;
    tft->fillRoundRect(lineX, lineY, lineW, 6, 3, COLOR_NAVY);

    tft->setFont(&FreeSans12pt7b);
    tft->setTextColor(ST77XX_WHITE);
    tft->setCursor(SCREEN_WIDTH / 2 + 30, SCREEN_HEIGHT / 2 - 10);
    tft->print("RPM");

    initializedScreenId = SCREEN_IDLE;
    prevSetRPM = -1;
    prevCommStatus = INVALID_STATUS;

    tft->setFont(&FreeSans9pt7b);
    tft->getTextBounds("Purge", 0, 0, &x1, &y1, &w, &h);

    tft->setCursor(SCREEN_WIDTH - 10 - w, SCREEN_HEIGHT*0.82);
    tft->setTextColor(ST77XX_WHITE);  
    tft->print("Purge");

    tft->setCursor(10, SCREEN_HEIGHT*0.82);
    tft->print("Drive");
    
    const char * purgeStatus;
    if(Menu2[AUTO_PURGE_ENABLED].value == true) { 
      purgeStatus = "Enabled";
    } else purgeStatus = "Disabled";

    tft->getTextBounds(purgeStatus, 0, 0, &x1, &y1, &w, &h);
    int valX = SCREEN_WIDTH - 10 - w;
    int valY = SCREEN_HEIGHT*0.91;

    tft->setCursor(valX, valY);
    tft->print(purgeStatus);
  }

  // --- Update setRPM ---
  if (prevSetRPM != setRPM) {
    char newVal[10];
    sprintf(newVal, "%d", setRPM);

    tft->setFont(&FreeSans24pt7b);
    tft->getTextBounds("8888", 0, 0, &x1, &y1, &w, &h); // Max 4 digits
    int valX = SCREEN_WIDTH / 2 - w;
    int valY = SCREEN_HEIGHT / 2 - 10;

    tft->fillRect(valX - 4, valY - h, w + 16, h + 12, ST77XX_BLACK);

    tft->getTextBounds(newVal, 0, 0, &x1, &y1, &w, &h); // Max 4 digits
    tft->setTextColor(ST77XX_WHITE);
    valX = SCREEN_WIDTH / 2 - w;
    tft->setCursor(valX, valY);
    tft->print(newVal);

    prevSetRPM = setRPM;
  }

  // --- Update commStatus ---
  if (prevCommStatus != commStatus) {
    const char* newStatus = getCommStatusText(commStatus);

    tft->setFont(&FreeSans9pt7b);
    tft->getTextBounds("Disconnected", 0, 0, &x1, &y1, &w, &h);
    int statusY = SCREEN_HEIGHT * 0.91;
    int statusX = 10;
    tft->fillRect(statusX - 1, statusY - h, w + 2, h + 8, ST77XX_BLACK);
    tft->setTextColor(getCommStatusColor(commStatus));
    tft->setCursor(statusX, statusY);
    tft->print(newStatus);

    prevCommStatus = commStatus;
  }
}

void drawGbWIdleScreen() {
  static float prevSetWeight = -1000;
  static float prevCurrentWeight = -1000;
  static SYSTEM_STATUS prevCommStatus = INVALID_STATUS;
  static SCALE_STATUS prevScaleStatus = INVALID_SCALE_STATUS;

  if (initializedScreenId != SCREEN_GBW_IDLE) {
    tft->fillScreen(ST77XX_BLACK);

    int lineY = SCREEN_HEIGHT * 0.7;
    int lineW = SCREEN_WIDTH * 0.8;
    int lineX = (SCREEN_WIDTH - lineW) / 2;
    tft->fillRoundRect(lineX, lineY, lineW, 6, 3, COLOR_NAVY);

    initializedScreenId = SCREEN_GBW_IDLE;
    prevSetWeight = -1.0;
    prevCurrentWeight = -1.0;
    prevCommStatus = INVALID_STATUS;
    prevScaleStatus = INVALID_SCALE_STATUS;

    tft->setFont(&FreeSans9pt7b);
    int16_t x1, y1; uint16_t w, h;
    tft->getTextBounds("Scale", 0, 0, &x1, &y1, &w, &h);

    tft->setCursor(SCREEN_WIDTH - 10 - w, SCREEN_HEIGHT*0.82);
    tft->setTextColor(ST77XX_WHITE);  
    tft->print("Scale");

    tft->setCursor(10, SCREEN_HEIGHT*0.82);
    tft->print("Drive");
  }

  // --- SetWeight (centered) ---
  if (prevSetWeight != setWeight) {
    char newVal[10];
    sprintf(newVal, "%.1f g", setWeight/1000.0);

    tft->setFont(&FreeSans24pt7b);
    int16_t x1, y1;
    uint16_t w, h;
    tft->getTextBounds("88.8 g", 0, 0, &x1, &y1, &w, &h);
    int valX = SCREEN_WIDTH/2 - w/2;
    int valY = SCREEN_HEIGHT / 2 - 30;
    tft->fillRect(valX - 2, valY - h, w + 10, h + 14, ST77XX_BLACK);

    tft->setTextColor(ST77XX_WHITE);
    tft->setCursor(valX, valY);
    tft->print(newVal);
    prevSetWeight = setWeight;
  }

  // --- CurrentWeight (right lower) ---
  if (prevCurrentWeight != currentWeight) {
    char newVal[10];
    sprintf(newVal, "%.1f g", (currentWeight / 1000.0));

    tft->setFont(&FreeSans12pt7b);
    int16_t x1, y1;
    uint16_t w, h;
    tft->getTextBounds("-888.8 g", 0, 0, &x1, &y1, &w, &h);
    int valX = SCREEN_WIDTH/2 + 40;
    int valY = SCREEN_HEIGHT*0.7 - 12;

    tft->fillRect(valX - 4, valY - h, w + 10, h + 12, ST77XX_BLACK);

    tft->setTextColor(getScaleStatusColor(scaleStatus));
    tft->setCursor(valX, valY);
    tft->print(newVal);

    prevCurrentWeight = currentWeight;
  }

  // --- COMM status (bottom-left, larger font, more right, centered) ---
  if (prevCommStatus != commStatus) {
    const char* newStatus = getCommStatusText(commStatus);

    tft->setFont(&FreeSans9pt7b);
    int16_t x1, y1; uint16_t w, h;
    tft->getTextBounds("Disconnected", 0, 0, &x1, &y1, &w, &h);
    int statusY = SCREEN_HEIGHT * 0.91;
    int statusX = 10;
    tft->fillRect(statusX - 1, statusY - h, w + 2, h + 8, ST77XX_BLACK);
    tft->setTextColor(getCommStatusColor(commStatus));
    tft->setCursor(statusX, statusY);
    tft->print(newStatus);

    prevCommStatus = commStatus;
  }

  // --- Scale status (bottom-right) ---
  if (prevScaleStatus != scaleStatus) {
    const char* newScaleText = getScaleStatusText(scaleStatus);

    tft->setFont(&FreeSans9pt7b);
    int16_t x1, y1; uint16_t w, h;
    tft->getTextBounds("Disconnected", 0, 0, &x1, &y1, &w, &h);
    int statusY = SCREEN_HEIGHT * 0.91;
    int statusX = SCREEN_WIDTH - 10 - w;
    tft->fillRect(statusX - 4, statusY - h, w + 10, h + 8, ST77XX_BLACK);

    tft->getTextBounds(newScaleText, 0, 0, &x1, &y1, &w, &h);
    statusY = SCREEN_HEIGHT * 0.91;
    statusX = SCREEN_WIDTH - 10 - w;

    tft->setTextColor(getScaleStatusColor(scaleStatus));
    tft->setCursor(statusX, statusY);
    tft->print(newScaleText);

    prevScaleStatus = scaleStatus;

    // Update weight too. 
    char newVal[10];
    sprintf(newVal, "%.1f g", (currentWeight / 1000.0));

    tft->setFont(&FreeSans12pt7b);
    tft->getTextBounds("-888.8 g", 0, 0, &x1, &y1, &w, &h);
    int valX = SCREEN_WIDTH/2 + 40;
    int valY = SCREEN_HEIGHT*0.7 - 12;

    tft->fillRect(valX - 4, valY - h, w + 10, h + 12, ST77XX_BLACK);

    tft->setTextColor(getScaleStatusColor(scaleStatus));
    tft->setCursor(valX, valY);
    tft->print(newVal);

    prevCurrentWeight = currentWeight;
  }
}

void drawGrindingOrPurgingScreen() {
    static int16_t prevRPM = -32768;
    static int16_t prevTorque = 0xFFFF;
    static SYSTEM_STATUS prevCommStatus = INVALID_STATUS;
    static bool prevPurgeEnabled = false;
    static bool prevPurgeReady = false;

    int16_t x1, y1;
    uint16_t w, h;

    // Layout constants
    const int dividerY = SCREEN_HEIGHT * 0.7;
    const int dividerW = SCREEN_WIDTH * 0.8;
    const int dividerX = (SCREEN_WIDTH - dividerW) / 2;
    const int speedoCX = SCREEN_WIDTH - SCREEN_WIDTH / 4;
    const int speedoCY = dividerY / 2 + 10;
    const int speedoR = 60;

    int thisScreenId = SCREEN_GRINDING;

    if (initializedScreenId != SCREEN_GRINDING && initializedScreenId != SCREEN_PURGING) {
        tft->fillScreen(ST77XX_BLACK);
        tft->fillRoundRect(dividerX, dividerY, dividerW, 6, 3, COLOR_NAVY);
        initializedScreenId = SCREEN_GRINDING;
        prevRPM = -32768;
        prevTorque = 0xFFFF;
        prevCommStatus = INVALID_STATUS;
        prevPurgeEnabled = !Menu2[AUTO_PURGE_ENABLED].value;
        prevPurgeReady = !ready_purge;

        tft->setFont(&FreeSans9pt7b);
        tft->getTextBounds("Purge", 0, 0, &x1, &y1, &w, &h);

        tft->setCursor(SCREEN_WIDTH - 10 - w, SCREEN_HEIGHT*0.82);
        tft->setTextColor(ST77XX_WHITE);  
        tft->print("Purge");

        tft->setCursor(10, SCREEN_HEIGHT*0.82);
        tft->print("Drive");
    }

    // --- RPM digits (left side) ---
    if (prevRPM != motor_currentRPM) {
      char newVal[10];
      sprintf(newVal, "%d", motor_currentRPM);

      tft->setFont(&FreeSans24pt7b);
      int16_t x1, y1;
      uint16_t w, h;
      tft->getTextBounds("-8888", 0, 0, &x1, &y1, &w, &h); // Max 4 digits
      int valX = 10;
      int valY = SCREEN_HEIGHT / 2 - 10;

      tft->fillRect(valX - 2, valY - h, w + 10, h + 8, ST77XX_BLACK);

      tft->setTextColor(ST77XX_WHITE);
      tft->setCursor(valX, valY);
      tft->print(newVal);

      prevRPM = motor_currentRPM;
    }
  

    // --- Torque speedometer (right side) ---
   
    int8_t torquePercent = (100 * motor_currentTorque / Menu1[SETMOTORTORQUE].value);
    //uint8_t torquePercent = 60;
    if (prevTorque != motor_currentTorque) {
        tft->fillCircle(speedoCX, speedoCY, speedoR + 2, ST77XX_BLACK);

        // Gradient color: green (0%) -> yellow (50%) -> red (100%)
        uint8_t r, g, b;
        if (torquePercent <= 50) {
            // Green to Yellow: (0,255,0) -> (255,255,0)
            r = (uint8_t)(255.0 * (torquePercent / 50.0));
            g = 255;
            b = 0;
        } else {
            // Yellow to Red: (255,255,0) -> (255,0,0)
            r = 255;
            g = (uint8_t)(255.0 * (1.0 - (torquePercent - 50) / 50.0));
            b = 0;
        }
        uint16_t color = tft->color565(r, g, b);

        float angle = (torquePercent / 100.0) * 270.0;
        for (int i = 0; i < angle; i += 4) {
            float rad = (135 + i) * M_PI / 180.0;
            int x0 = speedoCX + cos(rad) * (speedoR - 10);
            int y0 = speedoCY + sin(rad) * (speedoR - 10);
            int x1 = speedoCX + cos(rad) * speedoR;
            int y1 = speedoCY + sin(rad) * speedoR;
            tft->drawLine(x0, y0, x1, y1, color);
        }
        tft->drawCircle(speedoCX, speedoCY, speedoR, ST77XX_WHITE);

        tft->setFont(&FreeSans18pt7b);
        tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        char percentStr[8];
        sprintf(percentStr, "%d%%", torquePercent);
        int16_t x1, y1;
        uint16_t w, h;
        tft->getTextBounds(percentStr, 0, 0, &x1, &y1, &w, &h);
        tft->setCursor(speedoCX - w / 2, speedoCY+ h/2);
        tft->print(percentStr);

        prevTorque = motor_currentTorque;
    }

    // --- Connection status (bottom left) ---
  if (prevCommStatus != commStatus) {
    const char* newStatus = getCommStatusText(commStatus);

    tft->setFont(&FreeSans9pt7b);
    int16_t x1, y1; uint16_t w, h;
    tft->getTextBounds("Disconnected", 0, 0, &x1, &y1, &w, &h);
    int statusY = SCREEN_HEIGHT * 0.91;
    int statusX = 10;
    tft->fillRect(statusX - 1, statusY - h, w + 2, h + 8, ST77XX_BLACK);
    tft->setTextColor(getCommStatusColor(commStatus));
    tft->setCursor(statusX, statusY);
    tft->print(newStatus);

    prevCommStatus = commStatus;
  }

  const char* newPurgeText;

    // --- Purge status (bottom right) ---
    static bool prevPurgingDrawn = false;
    if (state == PURGING) {
        // Always show "Purging" in yellow
        if (!prevPurgingDrawn || initializedScreenId != SCREEN_PURGING) {
            
            newPurgeText = "Purging";

            tft->setFont(&FreeSans9pt7b);
            int16_t x1, y1; uint16_t w, h;
            tft->getTextBounds("Detecting..", 0, 0, &x1, &y1, &w, &h);
            int statusY = SCREEN_HEIGHT * 0.91;
            int statusX = SCREEN_WIDTH - 10 - w;
            tft->fillRect(statusX - 4, statusY - h, w + 2, h + 8, ST77XX_BLACK);

            tft->getTextBounds(newPurgeText, 0, 0, &x1, &y1, &w, &h);
            statusY = SCREEN_HEIGHT * 0.91;
            statusX = SCREEN_WIDTH - 10 - w;

            tft->setTextColor(ST77XX_WHITE);
            tft->setCursor(statusX, statusY);
            tft->print(newPurgeText);
            prevPurgingDrawn = true;
        }

    } else {
        // Show regular purge status
        if (prevPurgeEnabled != Menu2[AUTO_PURGE_ENABLED].value || prevPurgeReady != ready_purge || initializedScreenId != SCREEN_GRINDING) {
            tft->setFont(&FreeSans9pt7b);

            
            int16_t x1, y1; uint16_t w, h;
            tft->getTextBounds("Detecting..", 0, 0, &x1, &y1, &w, &h);
            int statusY = SCREEN_HEIGHT * 0.91;
            int statusX = SCREEN_WIDTH - 10 - w;
            tft->fillRect(statusX - 4, statusY - h, w + 2, h + 8, ST77XX_BLACK);

            tft->setTextColor(Menu2[AUTO_PURGE_ENABLED].value ? (ready_purge ? COLOR_GREEN : COLOR_YELLOW) : ST77XX_WHITE);

            if (Menu2[AUTO_PURGE_ENABLED].value == true) { 
              if (ready_purge == true) newPurgeText = "Imminent!";
              else newPurgeText = "Detecting..";
            } else newPurgeText = "Disabled";

                        tft->getTextBounds(newPurgeText, 0, 0, &x1, &y1, &w, &h);
            statusY = SCREEN_HEIGHT * 0.91;
            statusX = SCREEN_WIDTH - 10 - w;

            tft->setTextColor(ST77XX_WHITE);
            tft->setCursor(statusX, statusY);
            tft->print(newPurgeText);

            prevPurgeEnabled = Menu2[AUTO_PURGE_ENABLED].value;
            prevPurgeReady = ready_purge;
            prevPurgingDrawn = false;
        }
    }
}

void drawGrindingGbwScreen() {
    static int16_t prevRPM = -32768;
    static int32_t prevCurrentWeight = -999999;
    static SYSTEM_STATUS prevCommStatus = INVALID_STATUS;
    static SCALE_STATUS prevScaleStatus = INVALID_SCALE_STATUS;

    // Layout constants
    const int dividerY = SCREEN_HEIGHT * 0.7;
    const int dividerW = SCREEN_WIDTH * 0.8;
    const int dividerX = (SCREEN_WIDTH - dividerW) / 2;
    const int rpmX = 20;
    const int rpmY = dividerY / 2 - 32;
    const int speedoCX = SCREEN_WIDTH - SCREEN_WIDTH / 4;
    const int speedoCY = dividerY / 2 + 10;
    const int speedoR = 60;

    int thisScreenId = SCREEN_GRINDING_GBW;
    if (initializedScreenId != thisScreenId) {
        tft->fillScreen(ST77XX_BLACK);
        tft->fillRoundRect(dividerX, dividerY, dividerW, 6, 3, COLOR_NAVY);

        tft->setFont(&FreeSans9pt7b);
        int16_t x1, y1; uint16_t w, h;
        tft->getTextBounds("Scale", 0, 0, &x1, &y1, &w, &h);

        tft->setCursor(SCREEN_WIDTH - 10 - w, SCREEN_HEIGHT*0.82);
        tft->setTextColor(ST77XX_WHITE);  
        tft->print("Scale");

        tft->setCursor(10, SCREEN_HEIGHT*0.82);
        tft->print("Drive");

        initializedScreenId = thisScreenId;
        prevRPM = -32768;
        prevCurrentWeight = -999999;
        prevCommStatus = INVALID_STATUS;
        prevScaleStatus = INVALID_SCALE_STATUS;
    }

    // --- RPM digits (left side) ---
      if (prevRPM != motor_setRPM) {
      char newVal[10];
      sprintf(newVal, "%d", motor_setRPM);

      tft->setFont(&FreeSans24pt7b);
      int16_t x1, y1;
      uint16_t w, h;
      tft->getTextBounds("8888", 0, 0, &x1, &y1, &w, &h); // Max 4 digits
      int valX = 20;
      int valY = SCREEN_HEIGHT / 2 - 10;

      tft->fillRect(valX - 2, valY - h, w + 4, h + 8, ST77XX_BLACK);

      tft->setTextColor(ST77XX_WHITE);
      tft->setCursor(valX, valY);
      tft->print(newVal);

      prevRPM = motor_setRPM;
    }

    // --- Speedometer (right side): progress from 0 to setWeight ---
    int32_t weight = currentWeight;
    int32_t goal = setWeight;
    if (goal == 0) goal = 1; // avoid div by zero
    float percent = constrain(float(weight) / float(goal), 0.0, 1.0);

    if (prevCurrentWeight != weight) {
        tft->fillCircle(speedoCX, speedoCY, speedoR + 2, ST77XX_BLACK);

        // Color: linear from white to green
        uint8_t g = 0xE9 * percent + 0xFF * (1 - percent); // green increases, white fades
        uint16_t color = tft->color565((uint8_t)(0xFF * (1 - percent)), g, (uint8_t)(0xFF * (1 - percent)));

        float angle = percent * 270.0;
        for (int i = 0; i < angle; i += 4) {
            float rad = (135 + i) * M_PI / 180.0;
            int x0 = speedoCX + cos(rad) * (speedoR - 10);
            int y0 = speedoCY + sin(rad) * (speedoR - 10);
            int x1 = speedoCX + cos(rad) * speedoR;
            int y1 = speedoCY + sin(rad) * speedoR;
            tft->drawLine(x0, y0, x1, y1, color);
        }
        tft->drawCircle(speedoCX, speedoCY, speedoR, ST77XX_WHITE);

        // Draw % in center
        tft->setFont(&FreeSans12pt7b);
        tft->setTextColor(color, ST77XX_BLACK);
        char weightStr[8];
        sprintf(weightStr, "%.1fg", (currentWeight / 1000.0));
        int16_t x1, y1;
        uint16_t w, h;
        tft->getTextBounds(weightStr, 0, 0, &x1, &y1, &w, &h);
        tft->setCursor(speedoCX - w / 2, speedoCY + h/2);
        tft->print(weightStr);

        prevCurrentWeight = weight;
    }

    // --- COMM status (bottom-left, larger font, more right, centered) ---
    if (prevCommStatus != commStatus) {
      const char* newStatus = getCommStatusText(commStatus);

      tft->setFont(&FreeSans9pt7b);
      int16_t x1, y1; uint16_t w, h;
      tft->getTextBounds("Disconnected", 0, 0, &x1, &y1, &w, &h);
      int statusY = SCREEN_HEIGHT * 0.91;
      int statusX = 10;
      tft->fillRect(statusX - 1, statusY - h, w + 2, h + 8, ST77XX_BLACK);
      tft->setTextColor(getCommStatusColor(commStatus));
      tft->setCursor(statusX, statusY);
      tft->print(newStatus);

      prevCommStatus = commStatus;
    }

  if (prevScaleStatus != scaleStatus) {
      const char* newScaleText = getScaleStatusText(scaleStatus);

      tft->setFont(&FreeSans9pt7b);
      int16_t x1, y1; uint16_t w, h;
      tft->getTextBounds("Connection Lost", 0, 0, &x1, &y1, &w, &h);
      int statusY = SCREEN_HEIGHT * 0.91;
      int statusX = SCREEN_WIDTH - 10 - w;
      tft->fillRect(statusX - 4, statusY - h, w + 2, h + 8, ST77XX_BLACK);

      tft->getTextBounds(newScaleText, 0, 0, &x1, &y1, &w, &h);
      statusY = SCREEN_HEIGHT * 0.91;
      statusX = SCREEN_WIDTH - 10 - w;

      tft->setTextColor(getScaleStatusColor(scaleStatus));
      tft->setCursor(statusX, statusY);
      tft->print(newScaleText);

      prevScaleStatus = scaleStatus;
  }
}

void drawCalibratingScreen() {
    static int16_t prevRPM = -32768;
    static int16_t prevTorque = 0xFFFF;
    static SYSTEM_STATUS prevCommStatus = INVALID_STATUS;
    static int prevStep = -1;
    static int prevTotal = -1;

    // Layout constants
    const int dividerY = SCREEN_HEIGHT * 0.7;
    const int dividerW = SCREEN_WIDTH * 0.8;
    const int dividerX = (SCREEN_WIDTH - dividerW) / 2;
    const int speedoCX = SCREEN_WIDTH - SCREEN_WIDTH / 4;
    const int speedoCY = dividerY / 2 + 10;
    const int speedoR = 60;


    // Use global variables directly, do NOT redeclare or define them here!
    // calibrateArray and currentCal are declared extern in motorcontrol_rt.h
    int totalSteps = 1 + SIZEOFCALIBRATEARRAY / sizeof(uint16_t);
    int step = currentCal + 1; // Show as 1-based index

    // Draw divider and clear if needed
    if (initializedScreenId != SCREEN_CALIBRATING) {
        tft->fillScreen(ST77XX_BLACK);
        tft->fillRoundRect(dividerX, dividerY, dividerW, 6, 3, COLOR_NAVY);

        tft->setFont(&FreeSans9pt7b);
        tft->setTextColor(ST77XX_WHITE);  
        tft->setCursor(10, SCREEN_HEIGHT*0.82);
        tft->print("Drive");

        const char * calibrateText = "Calibrating.."; 
        tft->setFont(&FreeSans18pt7b);
        int16_t x1, y1;
        uint16_t w, h;
        tft->getTextBounds(calibrateText, 0, 0, &x1, &y1, &w, &h);
        tft->setCursor(SCREEN_WIDTH / 2 - w/2, 10 + h);
        tft->print(calibrateText);

        initializedScreenId = SCREEN_CALIBRATING;
        prevRPM = -32768;
        prevTorque = 0xFFFF;
        prevCommStatus = INVALID_STATUS;
        prevStep = -1;
        prevTotal = -1;
    }

    // --- RPM digits (left side) ---
    if (prevRPM != motor_currentRPM) {
      char newVal[10];
      sprintf(newVal, "%d", motor_currentRPM);

      tft->setFont(&FreeSans24pt7b);
      int16_t x1, y1;
      uint16_t w, h;
      tft->getTextBounds("8888", 0, 0, &x1, &y1, &w, &h); // Max 4 digits
      int valX = 10;
      int valY = SCREEN_HEIGHT / 2 - 10;

      tft->fillRect(valX - 2, valY - h, w + 10, h + 8, ST77XX_BLACK);

      tft->setTextColor(ST77XX_WHITE);
      tft->setCursor(valX, valY);
      tft->print(newVal);

      prevRPM = motor_currentRPM;
    }

    // --- Torque speedometer (right side) ---
    int16_t torquePercent = motor_currentTorque;
    //uint8_t torquePercent = 60;
    if (prevTorque != motor_currentTorque) {

        int16_t x1, y1;
        uint16_t w, h;
        tft->getTextBounds("8888", 0, 0, &x1, &y1, &w, &h); // Max 4 digits
        int valX = speedoCX - w / 2;
        int valY = speedoCY+ h/2;
        tft->fillRect(valX - 2, valY - h, w + 10, h + 8, ST77XX_BLACK);

        tft->setFont(&FreeSans18pt7b);
        tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        char percentStr[8];
        sprintf(percentStr, "%d", torquePercent);

        tft->getTextBounds(percentStr, 0, 0, &x1, &y1, &w, &h);
        tft->setCursor(speedoCX - w / 2, speedoCY+ h/2);
        tft->print(percentStr);

        prevTorque = motor_currentTorque;
    }


    // --- COMM status (bottom left) ---
    if (prevCommStatus != commStatus) {
      const char* newStatus = getCommStatusText(commStatus);

      tft->setFont(&FreeSans9pt7b);
      int16_t x1, y1; uint16_t w, h;
      tft->getTextBounds("Disconnected", 0, 0, &x1, &y1, &w, &h);
      int statusY = SCREEN_HEIGHT * 0.91;
      int statusX = 10;
      tft->fillRect(statusX - 1, statusY - h, w + 2, h + 8, ST77XX_BLACK);
      tft->setTextColor(getCommStatusColor(commStatus));
      tft->setCursor(statusX, statusY);
      tft->print(newStatus);

      prevCommStatus = commStatus;
    }

    // --- Calibration progress (bottom right) ---
    if (prevStep != step) {

        int16_t x1, y1;
        uint16_t w, h;
        tft->setFont(&FreeSans18pt7b);
        tft->setTextColor(ST77XX_WHITE);
        tft->getTextBounds("888 / 888", 0, 0, &x1, &y1, &w, &h);

        int statusX = SCREEN_WIDTH - 20 -w;
        int statusY = SCREEN_HEIGHT*0.9;
        tft->fillRect(statusX - 1, statusY - h, w + 12, h + 8, ST77XX_BLACK);

        char stepStr[16];
        sprintf(stepStr, "%d / %d", step, totalSteps);
        tft->getTextBounds(stepStr, 0, 0, &x1, &y1, &w, &h);
        tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft->setCursor(SCREEN_WIDTH - 20 - w, SCREEN_HEIGHT*0.90);
        tft->print(stepStr);

        prevStep = step;
    }
}

// Helper to get menu pointer by number
static const menuEntry* getMenuByNum(int menuNum) {
    switch(menuNum) {
        case 1: return Menu1;
        case 2: return Menu2;
        case 3: return Menu3;
        default: return nullptr;
    }
}

void drawMenuScreen(int menuNum, int numItems, int selectedIdx) {
    static int prevSelectedIdx = -1;
    static int prevMenuNum = -1;
    static char prevNames[5][21] = {0}; // 20 chars + null, for 5 visible items

    const menuEntry* menu = getMenuByNum(menuNum);

    // Layout constants
    const int itemCount = 5; // Always show 5 items
    const int centerY = SCREEN_HEIGHT / 2;
    const int itemHeight = 44;
    const int fadeColor = 0x8410; // 50% gray (faded)
    const int normalColor = ST77XX_WHITE;
    const int selectedColor = COLOR_GREEN;
    const int leftEdge = 10;
    const int centerX = SCREEN_WIDTH / 2;

    int thisScreenId = menuNum * 1000; // Use menuNum as unique ID

    bool fullRedraw = false;
    if (initializedScreenId != thisScreenId) {
        tft->fillScreen(ST77XX_BLACK);
        initializedScreenId = thisScreenId;
        prevSelectedIdx = -1; // force redraw
        prevMenuNum = menuNum;
        memset(prevNames, 0, sizeof(prevNames));
        fullRedraw = true;
    }

    if (prevSelectedIdx != selectedIdx) {
        fullRedraw = true;
        prevSelectedIdx = selectedIdx;
    }

    if (prevMenuNum != menuNum) {
        fullRedraw = true;
        prevMenuNum = menuNum;
    }

    for (int i = 0; i < itemCount; ++i) {
        int relIdx = i - itemCount / 2; // -2, -1, 0, 1, 2
        int menuIdx = (selectedIdx + relIdx + numItems) % numItems;

        int y = centerY + relIdx * itemHeight;

        // X position: move selected more to the left
        int x;
        if (relIdx == 0) x = leftEdge + 40;
        else if (abs(relIdx) == 1) x = leftEdge + 25;
        else x = leftEdge;

        uint16_t color = normalColor;
        if (relIdx == 0) color = selectedColor;
        else if (abs(relIdx) == 2) color = fadeColor;

        // Font: scale down selected, use FreeSans fonts
        const GFXfont* font;
        if (relIdx == 0) font = &FreeSans12pt7b;
        else font = &FreeSans9pt7b;

        char currName[21];
        strncpy(currName, menu[menuIdx].name, 20);
        currName[20] = '\0';

        char* nl = strchr(currName, '\n');
        if (nl) *nl = '\0';

        bool needsUpdate = fullRedraw || strcmp(prevNames[i], currName) != 0;

        if (needsUpdate) {
            int16_t x1, y1;
            uint16_t w, h;
            tft->setFont(font);
            tft->getTextBounds(prevNames[i], x, y, &x1, &y1, &w, &h);
            tft->fillRect(x1 - 10, y1 - 6, w + 20, h + 12, ST77XX_BLACK);

            tft->setFont(font);
            tft->setTextColor(color, ST77XX_BLACK);
            tft->setCursor(x, y);
            tft->print(currName);

            if (relIdx == 0) {
                tft->getTextBounds(currName, x, y, &x1, &y1, &w, &h);
                tft->drawRoundRect(x1 - 8, y1 - 4, w + 16, h + 8, 6, selectedColor);
            }

            strncpy(prevNames[i], currName, 20);
            prevNames[i][20] = '\0';
        }
    }
}

void drawMenuValueScreen(int menuNum, int selectedIdx) {
    // Clear screen if needed
    static int prevSelectedIdx = -1, prevMenuNum = -1;
    static int16_t prevValue = 0xFFFF;
    int thisScreenId = menuNum * selectedIdx * 32;
    
    const menuEntry* menu = getMenuByNum(menuNum);
    const menuEntry& item = menu[selectedIdx];

    if (initializedScreenId != thisScreenId) {
        tft->fillScreen(ST77XX_BLACK);
        initializedScreenId = thisScreenId;
        prevSelectedIdx = 0xFFFF;
        prevMenuNum = 0xFFFF;
        prevValue = 0xFFFF; // invalid so redraw

      // Special case: Calibrate (Menu1[CALIBRATE])
      if (menuNum == 1 && selectedIdx == 4) {
        // "Calibrate?" top center
        tft->setFont(&FreeSans18pt7b);
        tft->setTextColor(ST77XX_WHITE);
        int16_t x1, y1; uint16_t w, h;
        tft->getTextBounds("Calibrate?", 0, 0, &x1, &y1, &w, &h);
        tft->setCursor((SCREEN_WIDTH - w) / 2, 20 + h);
        tft->print("Calibrate?");
        
        // Instruction below
        tft->setFont(&FreeSans9pt7b);
        const char* msg1 = "Long press the encoder button to";
        const char* msg2 = "start the calibration process";
        const char* msg3 = "Press Start to exit";
        tft->getTextBounds(msg1, 0, 0, &x1, &y1, &w, &h);
        tft->setCursor((SCREEN_WIDTH - w) / 2, 100);
        tft->print(msg1);
        tft->getTextBounds(msg2, 0, 0, &x1, &y1, &w, &h);
        tft->setCursor((SCREEN_WIDTH - w) / 2, 120);
        tft->print(msg2);
        tft->getTextBounds(msg3, 0, 0, &x1, &y1, &w, &h);
        tft->setCursor((SCREEN_WIDTH - w) / 2, 140);
        tft->print(msg3);
      } else { 
          // Show menu item name, top center
          tft->setFont(&FreeSans12pt7b);
          tft->setTextColor(ST77XX_WHITE);
          int16_t x1, y1; uint16_t w, h;
          tft->getTextBounds(item.name, 0, 0, &x1, &y1, &w, &h);
          tft->setCursor((SCREEN_WIDTH - w) / 2, 30);
          tft->print(item.name);
      }
    } 
    if(menuNum == 1 && selectedIdx == 4) prevValue = item.value; //basically do nothing
    else {
      if(prevValue != item.value) {
        tft->fillRect(0, 60, SCREEN_WIDTH, SCREEN_HEIGHT-60, ST77XX_BLACK);
        // If min/max are 0/1, show "Disabled"/"Enabled" with box around selected
        prevValue = item.value;

        if (item.minValue == 0 && item.maxValue == 1 && item.scalar == 1) {
            const char* labels[2] = {"Disabled", "Enabled"};
            const char* alt_labels[2] = {"Don't save", "Save"};
            int selected = (item.value == 0) ? 0 : 1;
            String macCopy = "", nameCopy = "";
            if (xSemaphoreTake(scaleMutex, portMAX_DELAY)) {
                macCopy = scale_mac;
                nameCopy = scale_name;
                xSemaphoreGive(scaleMutex);
            } 

            for (int i = 0; i < 2; ++i) {
                const GFXfont* font = (i == selected) ? &FreeSans18pt7b : &FreeSans12pt7b;
                tft->setFont(font);
                tft->setTextColor(ST77XX_WHITE);
                int16_t x1, y1; uint16_t w, h;
                // Special case: Save scale (menu3.7)
                if(menuNum == 3 && selectedIdx == 7) {
                  tft->getTextBounds(alt_labels[i], 0, 0, &x1, &y1, &w, &h);

                  int y = 100 + i * 50;
                  tft->setCursor((SCREEN_WIDTH - w) / 2, y);
                  tft->print(alt_labels[i]);
                  if (i == selected) {
                      tft->drawRoundRect((SCREEN_WIDTH - w) / 2 - 8, y - h - 6 , w + 22, h + 16, 6, COLOR_GREEN);
                  }

                  if(i == 0) {
                    const char* msg4 = "last:";
                    tft->setFont(&FreeSans9pt7b);

                    int16_t x1, y1; uint16_t w, h;
                
                    tft->setCursor(10, 180);
                    tft->print(msg4);

                    tft->getTextBounds(msg4, 0, 0, &x1, &y1, &w, &h);
                    tft->setCursor(w + 30, 180);
                    tft->print(nameCopy);

                    tft->setFont(); // basic small font
                    tft->getTextBounds(macCopy, 0, 0, &x1, &y1, &w, &h);
                    tft->setCursor((SCREEN_WIDTH - w) / 2, 200);
                    tft->print(macCopy); 
                  } 
                } else 
                { // normal case
                  tft->getTextBounds(labels[i], 0, 0, &x1, &y1, &w, &h);

                  int y = 100 + i * 50;
                  tft->setCursor((SCREEN_WIDTH - w) / 2, y);
                  tft->print(labels[i]);
                  if (i == selected) {
                      tft->drawRoundRect((SCREEN_WIDTH - w) / 2 - 8, y - h - 6 , w + 22, h + 16, 6, COLOR_GREEN);
                  }
                }
            }

        } else {
            // Show value, big and centered
            char valStr[16];
            sprintf(valStr, "%d", item.value);
            tft->setFont(&FreeSans24pt7b);
            tft->setTextColor(COLOR_GREEN);
            int16_t x1, y1; uint16_t w, h;
            tft->getTextBounds(valStr, 0, 0, &x1, &y1, &w, &h);
            tft->setCursor((SCREEN_WIDTH - w) / 2, SCREEN_HEIGHT / 2);
            tft->print(valStr);
        }
      }
    }
} 
