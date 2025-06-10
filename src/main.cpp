#include <driver/rtc_io.h>
#include "main.h"
#include "gbw.h"
#include "pdata.h"
#include "display.h"
#include "io.h"

#ifdef RT_DRIVE 
#include "comm_rt.h"
#include "motorcontrol_rt.h"
#endif 
#ifdef JMC_DRIVE
#include "comm_jmc.h"
#include "motorcontrol_jmc.h"
#endif

uint8_t state = IDLE;
uint8_t commStatus = DISCONNECTED;
int8_t menu1Selected = EXITMENU;
int8_t menu2Selected = RETURN_FROM_PURGE;
int8_t menu3Selected = RETURN_FROM_GBW;
bool enterMenu = false;
int16_t menu1Value[] = {0};
int16_t menu2Value[] = {0};
int16_t menu3Value[] = {0};
bool newData = true;
bool purge_enabled = true;
uint8_t error = 0;
uint16_t sleepTime = default_sleeptime;
int16_t setRPM = default_setRPM;
uint16_t setWeight = default_weight;
bool grindingComplete = false;

unsigned long lastActivity = 0;

RTC_DATA_ATTR uint8_t rtc_bootFlag1 = 0;
RTC_DATA_ATTR bool modeGbW = false;
RTC_DATA_ATTR uint8_t rtc_state;
RTC_DATA_ATTR uint16_t rtc_setRPM;

void setup() { 
    setCpuFrequencyMhz(80);
    #ifdef DEBUG
        Serial.begin(115200);
    #endif
    
    if(rtc_bootFlag1 == 1) { 
        sleep_release();
       // display_wakey(); 
    }
    comm_init();
    motor_init();
    vitals_init();
    scales_init(); 
    pdata_init();
    display_init();
    pdata_read();
 
    lastActivity = millis();
}

void loop() { 
    static unsigned long lastUpdate = 0;

    doVitals();
    do_comm();
    update_display();
    check_io();

    if(state == SLEEPING){ 
        ledAction(0);
        display_off();
        sleep_init();
        esp_deep_sleep_start();
    }
    
    if(state == IDLE)
    { 
        ledAction(2);
        motor_setRPM = 0;
    }

    if(state == IDLE_GBW) { 
        ledAction(2);
        motor_setRPM = 0;
    }

    if(state == GRINDING)
    { 
        checkPurge();
    }

    if(state == PURGING) { 

    }

    if(state == GRINDING_GBW)
    { 
        gbwVitals(); 
        // Completion route
        if(gbw_predict() < COMMINTERVAL && grindingComplete == false) { 
            motorOff();
            grindingComplete = true;
            lastUpdate = millis();
        }

        // Disconnected route
        if(scale_connected() == false && grindingComplete == false) { 
            motorOff();
            state = IDLE_GBW;
            error = 104;
        // Still working on it route
        } 
        
        if(scale_connected() == true && grindingComplete == false) { 
            motor_setRPM = GBW_RPM_SET;
            ledAction(2);
        }

        // Aftermath route
        if(grindingComplete == true && lastUpdate + 500 > millis()) { 
            gbw_learn();
            ledAction(4);
        }

        // exit route
        if(grindingComplete == true && lastUpdate + 500 < millis()) { 
            state = IDLE_GBW;
            grindingComplete = false;
        }
    }

    if(state == MENU1)
    { 

    }

    if(state == CALIBRATING) { 
        if(check_io() == true) { 
            motorOff();
            error = 102;
            state = MENU1;
        } else {
        ledAction(1);
        Calibrate();
        }
    }

    newData = false; // all parts of the loop have received the data
}


void checkPurge() { 
    
}

void vitals_init() { 
  // watchdog??
  //pdata?
}


void sleep_init() { 
    digitalWrite(RS485RE, HIGH); //This sets the RS485 chip to a low power state
    digitalWrite(RS485DE, LOW); 
    rtc_gpio_deinit(WAKE_BTN); // Init buttons to wake up from
    rtc_gpio_deinit(WAKE_ENC_BTN);
    rtc_gpio_deinit(WAKE_ENC_A);
    rtc_gpio_deinit(WAKE_ENC_B);
    uint8_t rtc_state = state; // ----- Need this to be something like " last mode ie GbW, timed, or normal "  TO DO -----
    uint16_t rtc_setRPM = setRPM; //This is the last set RPM;
    delay(1);
    rtc_gpio_hold_en(SLEEP_RE);    // Hold the pin state during deep sleep
    rtc_gpio_hold_en(SLEEP_DE);    // Hold the pin state during deep sleep
    esp_sleep_enable_ext0_wakeup(WAKE_BTN, 1); // --- Currently HIGH not sure this is the final implementation
    esp_sleep_enable_ext0_wakeup(WAKE_ENC_BTN, 1);
    esp_sleep_enable_ext0_wakeup(WAKE_ENC_A, !digitalRead(ENC_A)); // not quite sure this works
    esp_sleep_enable_ext0_wakeup(WAKE_ENC_B, !digitalRead(ENC_B));
}

void sleep_release() { 
    rtc_gpio_hold_dis(WAKE_BTN);
    rtc_gpio_hold_dis(WAKE_ENC_BTN);
    rtc_gpio_hold_dis(WAKE_ENC_A);
    rtc_gpio_hold_dis(WAKE_ENC_B);
    rtc_gpio_hold_dis(SLEEP_RE);
    rtc_gpio_hold_dis(SLEEP_DE);
}


void doVitals() { 
    //watchdog?

}


void updateSleepTimer() { 
    
}
