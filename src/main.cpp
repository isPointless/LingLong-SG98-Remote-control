#include <driver/rtc_io.h>
#include "esp_task_wdt.h"
#include "display.h"
#include "main.h"
#include "gbw.h"
#include "pdata.h"
#include "display.h"
#include "io.h"

#define DEBUG

#ifdef RT_DRIVE 
#include "comm_rt.h"
#include "motorcontrol_rt.h"
#endif 
#ifdef JMC_DRIVE
#include "comm_jmc.h"
#include "motorcontrol_jmc.h"
#endif

uint8_t state = IDLE;
SYSTEM_STATUS commStatus = DISCONNECTED;
int8_t menu1Selected = EXITMENU;
int8_t menu2Selected = RETURN_FROM_PURGE;
int8_t menu3Selected = RETURN_FROM_GBW;
bool enterMenu = false;
bool firstBootFlag = false;

menuEntry Menu1[NUM_MENU1_ITEMS] = { 
    {"EXIT MENU", 0, 0, 0, 0}, //0
    {"Brightness", default_brightness, 1, 255, 1}, //1
    {"Purge Settings", 0, 0, 0, 0}, //2
    {"GbW Settings", 0, 0, 0, 0}, //3
    {"Calibrate", 0, 0 ,0, 0}, //4
    {"Max RPM", default_maxRPM, absolute_min_rpm, absolute_max_rpm, rpm_scalar}, //5
    {"Min RPM", default_minRPM, absolute_min_rpm, 500, rpm_scalar}, //6
    {"Sleep time", default_sleepTime, 0, 1800, 30}, //7
};
menuEntry Menu2[NUM_MENU2_ITEMS] = { 
    {"BACK", 0, 0, 0}, //0
    {"Purge /w button", default_buttonPurge, 0, 1, 1}, //1
    {"Purge frames LOW", default_purgeFramesLow, 2, 20, 1}, //2
    {"Purge frames HIGH", default_purgeFramesHigh, 2, 20, 1}, //3
    {"Purge Scalar LOW", default_purgePrctLow, 101, 200, 1}, //4
    {"Purge Scalar HIGH", default_purgePrctHigh, 101, 200, 1}, //5
    {"Purge duration", default_purgeDuration, 500, 5000, 100,}, //6
    {"Purge delay", default_purgeDelay, 100, 10000, 50}, //7
    {"Reverse rotation", default_reverseRotation, absolute_min_rpm, absolute_max_rpm, rpm_scalar}, //8
    {"Auto purge enabled", default_autoPurgeEnabled, 0, 1, 1}, //9
    {"Purge stabilize time", default_purgeStabilTime, 100, 5000, 50}, //10
    {"Purge forward RPM", default_purgeForwardRPM, absolute_min_rpm, absolute_max_rpm, rpm_scalar}, //11
};
menuEntry Menu3[NUM_MENU3_ITEMS] = { 
    {"BACK", 0, 0, 0, 0}, //0
    {"GbW RPM", default_GBWRPM, 200, 1500, rpm_scalar}, //1
    {"GbW Slow Phase t", default_slow_time, 1, 2000, 50}, //2
    {"GbW Slow RPM", default_slow_rpm, rpm_scalar, 500, rpm_scalar}, //3
    {"SpeedModifier", default_speedModifier, 1, 32767, 0}, //4 * 10.000
    {"Start delay", default_start_delay, 0, 1000, 50}, // 5
    {"Time offset", default_time_offset, 0, 1000, 10}, // 6
};

volatile bool newData = true;

volatile uint8_t error = 0;
int16_t setRPM = default_setRPM;
uint32_t setWeight = default_setWeight;
volatile bool runGbwVitals = true;

unsigned long lastActivity = 0;

RTC_DATA_ATTR uint8_t rtc_bootFlag1 = 0;
RTC_DATA_ATTR bool modeGbW = false;
RTC_DATA_ATTR uint8_t rtc_state;
RTC_DATA_ATTR uint16_t rtc_setRPM;

bool ready_purge = false;

void scaleTask(void *pvParameters) {
    scales_init(); // Run once at task start
    while (1) {
        if (runGbwVitals) {
            gbwVitals();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() { 
    delay(2000);
    Serial.begin(115200);
    Serial.println("start");
    setCpuFrequencyMhz(80);
    
    xTaskCreatePinnedToCore(
    scaleTask,         // Task function
    "ScaleTask",       // Name
    4096,              // Stack size (adjust as needed)
    NULL,              // Parameter
    1,                 // Priority
    NULL,              // Task handle
    0                  // Core 0 (use 1 for core 1)
    );
    
    if(rtc_bootFlag1 == true) { 
        sleep_release();
    }

    #ifdef DEBUG
    Serial.print("Wakeup cause: ");
    Serial.println(esp_sleep_get_wakeup_cause());
    Serial.print("Reset reason: ");
    Serial.println(esp_reset_reason());
    #endif

    // comm_init();
    // Serial.println("Comm init");
    // motor_init();
    // Serial.println("motor_init");
    vitals_init();
    
    pdata_init(); //This sets state as well!
    Serial.println("pdata init");

    if(firstBootFlag == 1) { 
         pdata_write(0);
    } 

    display_init();
    Serial.println("display init");
    pdata_read();
    Serial.println("pdata read");

    lastActivity = millis();
    io_init();
}

void loop() { 
    doVitals();
    do_comm();
    do_io();
    sleepTimeCheck();

    if(newData == true) {
        update_display();
        newData = false;
        // #ifdef DEBUG
        //     Serial.print("newdata! state: ");
        //     Serial.print(state);
        //     Serial.print(" rpm: ");
        //     Serial.print(setRPM);
        //     Serial.print(" weight: ");
        //     Serial.print(setWeight);
        //     Serial.print(" M1: ");
        //     Serial.print(menu1Selected);
        //     Serial.print(" M2: ");
        //     Serial.print(menu2Selected);
        //     Serial.print(" M3: ");
        //     Serial.println(menu3Selected);

        //     Serial.print("Minimum free heap since boot: ");
        //     Serial.println(esp_get_minimum_free_heap_size());

        //     UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(NULL);
        //     Serial.print("Stack high water mark: ");
        //     Serial.println(stackRemaining);
        // #endif
    }

    if(state == SLEEPING){ 
        ledAction(0);
        motorOff();
        display_off();
        sleep_init();
    }
    
    if(state == IDLE)
    { 
        motor_setRPM = 0;
    }

    if(state == IDLE_GBW) { 
        motor_setRPM = 0;
        runGbwVitals = true;
    }

    if(state == GRINDING)
    { 
        motor_setRPM = setRPM;
        checkPurge();
        lastActivity = millis();
    }

    if(state == PURGING) { 
        ledAction(1);
        do_purge();
        lastActivity = millis();
    }

    if(state == GRINDING_GBW)
    { 
        runGbwVitals = true;
        do_gbw();
        lastActivity = millis();
    }

    if(state == MENU1)
    { 
        motor_setRPM = 0;
        //everything handled in do_io
    }

    if(state == MENU2) { 
        motor_setRPM = 0;
        //everything handled in do_io
    }

    if(state == MENU3) { 
        motor_setRPM = 0;
        //everything handled in do_io
    }

    if(state == CALIBRATING) { 
        // ledAction(1);
        // Calibrate();
        lastActivity = millis();
    }

}


void checkPurge() { 
    static uint8_t frameCounter = 0;
    static int16_t last_setRPM;
    static unsigned long startTime = 0;
    static unsigned long lastCall = 0;
    static float test_torque;
    static int16_t test_scalar;
    static int16_t purge_frames;

    //if last call was +200ms ago, reset.
    if(lastCall + 200 < millis()) { 
        lastCall = millis();
        startTime = millis();
        ready_purge = false;
        frameCounter = 0;
    }

    //Reset if the RPM is changed
    if(setRPM != last_setRPM) {
        last_setRPM = setRPM;
        frameCounter = 0;
        ready_purge = false;
        //lastActivity=millis(); < superfluous 
    }

    //value 9 = auto purge enabled, value 10 = auto purge calibration delay
    if(Menu2[9].value == true && startTime + Menu2[10].value < millis() && lastActivity + 1000 < millis() && newData == true) 
    { 
        //the torque defined during calibration
      test_torque = calibrateArray[((setRPM + absolute_min_rpm)/rpm_scalar)];

      if(setRPM > 500) { //High values
        test_scalar = Menu2[5].value;
        purge_frames = Menu2[3].value;
      } else {          // low values
        test_scalar = Menu2[4].value;
        purge_frames = Menu2[2].value;
      }

      ///if the actual torque is greater than the calibrated torque*scalar we count up frames (note only when newData = true)
      if(motor_currentTorque > (test_torque*test_scalar)/100.f) { 
        frameCounter++;
        #ifdef DEBUG_CALIBRATE
          SerialUSB.print("frame counter: "), SerialUSB.println(frameCounter);
        #endif
      } else {  //If it's smaller, we set the frames to 0;
        frameCounter = 0;
      }

      // If we've made the threshold once, purge will commence after grinding. LastActivity stay millis() as long as we're grinding beans
      if(frameCounter >= purge_frames)  
      { 
        ready_purge = true;
        lastActivity = millis();
      }
      
      //If the frames have gone to 0; we are no longer grinding -> after purgedelay(7) we will commense the purge
      if(ready_purge == true && (lastActivity + Menu2[7].value) < millis()) 
      { 
        #ifdef DEBUG_CALIBRATE
        SerialUSB.println("go purge");
        #endif
        newData = true;
        ready_purge = false;
        frameCounter = 0; //should already be 0 but ok.
        state = PURGING;
        }
    }

    lastCall = millis();
}

void do_purge() { 
    static unsigned long lastCall;
    static unsigned long startTime;
    static uint8_t phase;
    static uint8_t last_purge_phase; 
    const uint16_t delayTime = 1000;

    // if its the first call in a while..
    if(lastCall + 200 < millis()) { 
        lastCall = millis();
        startTime = millis(); 
        phase = 0;
        last_purge_phase = 0xFF;
        Serial.println("Purge commence..");
    }
    
    //started <500ms ago
    if(startTime + delayTime > millis()) { 
        motor_setRPM = 0; 
        phase = 1; 
    }
    // Time between 500 and 500 + purgeTime 
    if(startTime + delayTime < millis() && startTime + delayTime + Menu2[6].value > millis()) {
        motor_setRPM = Menu2[11].value; 
        phase = 2;
    }
    //We're not doing reverse
    if(Menu2[8].value == false && startTime + delayTime + Menu2[6].value < millis()) {
        motor_setRPM = 0;
        state = IDLE;
        newData = true;
        phase = 5;
    } else { 
        //were doing reverse, pause for 500ms 
        if(startTime + delayTime + Menu2[6].value < millis() && startTime + 2*delayTime + Menu2[6].value > millis()) { 
            motor_setRPM = 0;
            phase = 3;
        }
        //do another reverse purge for purge time
        if(startTime + 2*delayTime + Menu2[6].value < millis() && startTime + 2*delayTime + 2*Menu2[6].value > millis()) {
            motor_setRPM = (-1)*Menu2[8].value;
            phase = 4;
        }
        if(startTime + 2*delayTime + 2*Menu2[6].value < millis()) { 
            phase = 5;
            motor_setRPM = 0;
            state = IDLE;
            newData = true;
        }
    }

    if(phase != last_purge_phase) { 
        Serial.println(phase);
        newData = true;
        last_purge_phase = phase;
        Serial.println(motor_setRPM);
    }

    lastCall = millis();
}


void vitals_init() { 
  esp_task_wdt_init(2, true);
  esp_task_wdt_add(NULL); // Add current task (loop task)
}


void sleep_init() { 
    digitalWrite(RS485RE, HIGH); // Low power state
    digitalWrite(RS485DE, LOW); 
    digitalWrite(DISP_BL, LOW);

    delay(1);

    // Set BTN as input with pullup
    //pinMode(BTN, INPUT_PULLUP);

    rtc_bootFlag1 = true;

    // --- Enable RTC GPIO hold for RE, DE, DISP_BL ---
    rtc_gpio_hold_en((gpio_num_t)RS485RE);
    rtc_gpio_hold_en((gpio_num_t)RS485DE);
    rtc_gpio_hold_en((gpio_num_t)DISP_BL);


    rtc_gpio_hold_dis((gpio_num_t)BTN); // Release hold just in case
    rtc_gpio_init((gpio_num_t)BTN);
    rtc_gpio_set_direction((gpio_num_t)BTN, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en((gpio_num_t)BTN);

    // Configure EXT0 wakeup on BTN (GPIO 4), wake on LOW level (button press)
    esp_sleep_enable_ext0_wakeup(WAKE_BTN, 0); // 0 = wake on LOW

    Serial.println("Entering deep sleep, waiting for BTN press to wake up...");
    Serial.print("BTN: "); Serial.println(digitalRead(BTN));

    // Flush serial before sleeping
    Serial.flush();

    // Enter deep sleep (will not return until reset/wakeup)
    esp_deep_sleep_start();
}

void sleep_release() { 
   // --- Release RTC GPIO hold for RE, DE, DISP_BL ---
    rtc_gpio_hold_dis((gpio_num_t)RS485RE);
    rtc_gpio_hold_dis((gpio_num_t)RS485DE);
    rtc_gpio_hold_dis((gpio_num_t)DISP_BL);

    // Disable EXT0 wakeup source (optional, for clarity)
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);
    rtc_bootFlag1 = false;
}


void doVitals() { 
    esp_task_wdt_reset();
}


void sleepTimeCheck() { 
    if(Menu1[7].value == 0) {
        setBrightness();
        return;
    }
    else { 
        if(lastActivity + Menu1[7].value * 1000 < millis()) { 
            rtc_state = state; // save last state
            state = SLEEPING;
            Serial.println("sleep!");
        }
    }
}
