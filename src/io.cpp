#include "definitions.h"
#include "io.h"
#include "main.h"
#include <math.h>
#include <JC_Button.h>
#include <ESP32Encoder.h>
#include "pdata.h"

#ifdef JMC_DRIVE
#include "motorcontrol_jmc.h"
#endif

#ifdef RT_DRIVE
#include "motorcontrol_rt.h"
#endif


Button enc_button(ENC_BTN);
Button start_button(BTN);
ESP32Encoder encoder;

bool longPress = false;
int16_t storedSetRPM = 0;
uint32_t storedSetWeight = 0;

void io_init() { 
    pinMode(BTN, INPUT);
    pinMode(ENC_BTN, INPUT);
    pinMode(start_button_led, OUTPUT);

    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);
    
    encoder.attachHalfQuad(ENC_A, ENC_B);
    encoder.setFilter(100);
    encoder.setCount(0);

    ledcSetup(0,4000,8);
    ledcAttachPin(start_button_led, 0);
}

void do_io() { 
    static bool lastIdle = false; //lastIdle true means it was on normal Idle before, false means idle_GBW
    static int16_t OriginalValue = 0;
    static bool notReleasedYet = false;
    

    if(state == IDLE) { 
        ledAction(3);
        setRPM += encoder_change()*rpm_scalar;

        if(setRPM > Menu1[5].value) setRPM = Menu1[5].value;
        if(setRPM < Menu1[6].value) setRPM = Menu1[6].value;
        
        //START BUTTON
        if(START_BUTTON() == true && longPress == false) state = GRINDING;

        //ENC BUTTON
        if(ENC_BUTTON() == true) { 
            notReleasedYet = false;
            if(longPress == false) { 
                state = MENU1;
                lastIdle = true;
            }
        } else { // enc_button == false
            if(longPress == true && notReleasedYet == false) {  //when we're still holding longPress 
            state = IDLE_GBW;
            newData = true;
            notReleasedYet = true;
            pdata_write(5);
            }
        }
        //OTHER
        if(pressedLong() == true) { 
            longPress = true;
        } else longPress = false;

        if(setRPM != storedSetRPM && lastActivity + 10000 < millis()) {
            storedSetRPM = setRPM;
            pdata_write(3);
        }

    }

    if(state == IDLE_GBW) { 
        ledAction(3);
        setWeight += encoder_change() * 100;
        
        if(setWeight > default_max_weight) setWeight = default_max_weight;
        if(setWeight < default_min_weight) setWeight = default_min_weight;
        
        if(START_BUTTON() == true && longPress == false) state = GRINDING_GBW;

        //ENC BUTTON
        if(ENC_BUTTON() == true) { 
            notReleasedYet = false;
            if(longPress == false) { 
                state = MENU1;
                lastIdle = false;
            }
        } else { // enc_button == false
            if(longPress == true && notReleasedYet == false) {  //when we're still holding longPress 
            state = IDLE;
            newData = true;
            notReleasedYet = true;
            pdata_write(5);
            }
        }
        //OTHER
        if(pressedLong() == true) { 
            longPress = true;
        } else longPress = false;
        
        if(setWeight != storedSetWeight && lastActivity + 10000 < millis()) {
            storedSetWeight = setWeight;
            pdata_write(3);
        }

    }

    if(state == GRINDING) { 
        //Flash responsively based on auto purge.
        if(ready_purge == false) ledAction(2);
        else ledAction(4);

        setRPM += encoder_change()*rpm_scalar;

        if(setRPM > Menu1[5].value) setRPM = Menu1[5].value;
        if(setRPM < Menu1[6].value) setRPM = Menu1[6].value;

        if(START_BUTTON() == true) { 
            state = (Menu2[1].value == true? PURGING : IDLE);
        }
        if(ENC_BUTTON() == true) { 
            state = IDLE;
            motor_setRPM = 0;
        }
    }

    if(state == PURGING) { 
        ledAction(1);
        setRPM += encoder_change()*rpm_scalar;

        if(START_BUTTON() == true || ENC_BUTTON() == true) { 
            state = IDLE;
            motor_setRPM = 0;
        }
    }

    if(state == GRINDING_GBW) { 
        //setWeight += encoder_change();

        if(START_BUTTON() == true || ENC_BUTTON() == true) { 
            state = IDLE_GBW;
            motor_setRPM = 0;
        }

    }
    if(state == MENU1) { 
        //Encoder routes
        if(enterMenu == false) { 
            menu1Selected += encoder_change();
            ledAction(0);
        } 

        if(menu1Selected > NUM_MENU1_ITEMS-1) menu1Selected = 0;
        if(menu1Selected < 0) menu1Selected = NUM_MENU1_ITEMS - 1;

        if(enterMenu == true) { 
            Menu1[menu1Selected].value += encoder_change() * Menu1[menu1Selected].scalar;
            if(Menu1[menu1Selected].value > Menu1[menu1Selected].maxValue) Menu1[menu1Selected].value = Menu1[menu1Selected].maxValue;
            if(Menu1[menu1Selected].value < Menu1[menu1Selected].minValue) Menu1[menu1Selected].value = Menu1[menu1Selected].minValue;
            if(menu1Selected == CALIBRATE) { 
                //DISPLAY press start to continue
                ledAction(2);
            } else ledAction(0);
        }

        //Button routes
        if(START_BUTTON() == true) {
            if (enterMenu == false) { 
                menu1Selected = EXITMENU;
                lastIdle? state = IDLE : state = IDLE_GBW; 
            } else { 
                enterMenu = false;
                menu1Selected = EXITMENU;
                Menu1[menu1Selected].value = OriginalValue;
            }
        }

        if(ENC_BUTTON() == true) { 
            if(enterMenu == false) { 
                if(menu1Selected == PURGESETTINGS) state = MENU2;
                if(menu1Selected == GBWSETTINGS) state = MENU3;
                if(menu1Selected == EXITMENU) lastIdle? state = IDLE : state = IDLE_GBW; 
                if(menu1Selected != PURGESETTINGS && menu1Selected != GBWSETTINGS && menu1Selected != EXITMENU) enterMenu = true; //entering settings adjustment
                OriginalValue = Menu1[menu1Selected].value;
            } else { 
                enterMenu = false;
                pdata_write(2);
            }
        } else if(longPress == true && menu1Selected == CALIBRATE) { 
            state = CALIBRATING;  
            newData = true;
            notReleasedYet = true;
            }

        //OTHER
        if(pressedLong() == true) { 
            longPress = true;
        } else longPress = false;
        
    }

    if(state == MENU2) { 
        //ENCODER ROUTE
        if(enterMenu == false) { 
            menu2Selected += encoder_change();
            ledAction(0);
        }

        if(menu2Selected > NUM_MENU2_ITEMS - 1) menu2Selected = 0;
        if(menu2Selected < 0) menu2Selected = NUM_MENU2_ITEMS - 1;

        if(enterMenu == true) { 
            Menu2[menu2Selected].value += encoder_change() * Menu2[menu2Selected].scalar;
            if(Menu2[menu2Selected].value > Menu2[menu2Selected].maxValue) Menu2[menu2Selected].value = Menu2[menu2Selected].maxValue;
            if(Menu2[menu2Selected].value < Menu2[menu2Selected].minValue) Menu2[menu2Selected].value = Menu2[menu2Selected].minValue;
            
            if(menu2Selected == SETBUTTONPURGE && Menu2[1].value == true) { 
                ledAction(2);
            } else ledAction(0);
        }

        //BUTTON ROUTES
        if(START_BUTTON() == true) { 
            if(enterMenu == false) { 
                menu2Selected = RETURN_FROM_PURGE;
                state = MENU1;
            } else { 
                enterMenu = false;
                Menu2[menu2Selected].value = OriginalValue;
            }
        } 

        if(ENC_BUTTON() == true) { 
            if(enterMenu == false) { 
                if(menu2Selected == RETURN_FROM_PURGE) state = MENU1;
                if(menu2Selected != RETURN_FROM_PURGE) enterMenu = true;
                OriginalValue = Menu2[menu2Selected].value;
            } else { 
                enterMenu = false;
                pdata_write(2);
            }
        }
    }

    if(state == MENU3) { 
        ledAction(0);
        //Encoder routes
        if(enterMenu == false) menu3Selected += encoder_change();
        if(menu3Selected > NUM_MENU3_ITEMS - 1) menu3Selected = 0;
        if(menu3Selected < 0) menu3Selected = NUM_MENU3_ITEMS - 1;

        if(enterMenu == true) { 
            Menu3[menu3Selected].value += encoder_change()* Menu3[menu3Selected].scalar;
            if(Menu3[menu3Selected].value > Menu3[menu3Selected].maxValue) Menu3[menu3Selected].value = Menu3[menu3Selected].maxValue;
            if(Menu3[menu3Selected].value < Menu3[menu3Selected].minValue) Menu3[menu3Selected].value = Menu3[menu3Selected].minValue;
        }

        //Button routes
        if(START_BUTTON() == true) { 
            if(enterMenu == false) { 
                menu3Selected = RETURN_FROM_GBW;
                state = MENU1;
            } else { 
                enterMenu = false;
                Menu3[menu3Selected].value = OriginalValue;
            }
        } 

        if(ENC_BUTTON() == true) { 
            if(enterMenu == false) { 
                if(menu3Selected == RETURN_FROM_GBW) state = MENU1;
                if(menu3Selected != RETURN_FROM_GBW) enterMenu = true;
                OriginalValue = Menu3[menu3Selected].value;
            } else { 
                enterMenu = false;
                pdata_write(2);
            }
        } 
    }

    if(state == CALIBRATING) { 
        longPress = false;
        if(ENC_BUTTON() == true) { 
            if(notReleasedYet == true) notReleasedYet = false;
            else {
                motorOff();
                error = 102;
                state = MENU1;
            }
        }
        if(START_BUTTON() == true) {
            motorOff();
            error = 102;
            state = MENU1;
        }

        ledAction(1);
    }

}

bool pressedLong() { 
    if(enc_button.pressedFor(LONGPRESS)) {
        return true;
    }

    return false;
}

int16_t encoder_change() { 
    static int32_t lastV = 0;
    int32_t v = 0;
    int16_t change = 0;

    v = encoder.getCount()/ENC_TOL; 
    change = lastV - v;
    lastV = v;

    if(change != 0) { 
        lastActivity = millis();
        newData = true;
    }
    return change;
}

bool check_io() { //returns true immediately when a button is pressed - normal control uses wasReleased to prevent double feedback.
    if(enc_button.isPressed() || enc_button.wasReleased()) return true;
    if(start_button.isPressed() || start_button.wasReleased()) return true;
    return false;
}

bool ENC_BUTTON() { 
  enc_button.read();
    if (enc_button.wasReleased()) { 
        lastActivity = millis();
        newData = true;
        return true;
    }
    return false;
}

bool START_BUTTON() {
    start_button.read();
    if (start_button.wasReleased()) { 
        lastActivity = millis();
        newData = true;
        return true;
    }
    return false;
}

void ledAction(uint8_t type) {  //Type: 0 = off, 1 = on, 2 = flashing, 3 = fading, 4 flashing fast, 5 = flashing fastest, 100-200 = %.
    static unsigned long lastCall;
    static uint8_t lastType;
    static unsigned long lastFlash;
    static uint8_t lastLedValue;
    uint8_t ledValue;
    bool ledOn = true;
    
    if(lastCall + 10 > millis()) return; //does this really add anything..?

    if (type == 0) { 
        ledValue = 0;
    }

    if (type == 1) {
        ledValue = 255;
    }

    //flashing
    if (type == 2 || type == 4 || type == 5) { 

        //On if starting from flash
        if(lastType != type) { 
            ledOn = true;
            ledValue = 255;
            lastFlash = millis();
        }
        //flash
        if(type == 2) {
            if(lastFlash + flashTime < millis()) { 
                ledOn = false;
            }

            if(lastFlash + 2*flashTime < millis()) { 
                ledOn = true;
                lastFlash = millis();
            }
        }
        //faster
        if(type == 4) { 
            if(lastFlash + fastFlashTime < millis()) { 
                ledOn = false;
            }

            if(lastFlash + 2*fastFlashTime < millis()) { 
                ledOn = true;
                lastFlash = millis();
            }
        }
        //FASTER!!
        if(type == 5) { 
            if(lastFlash + fastestFlashTime < millis()) { 
                ledOn = false;
            }

            if(lastFlash + 2*fastestFlashTime < millis()) { 
                ledOn = true;
                lastFlash = millis();
            }            
        }

        if(ledOn == true) ledValue = 255;
        else ledValue = 0;
    }

    if (type == 3) { //fading
        static float cosValue = 0;

        //Start from last value
        if (lastType != type) { 
            lastType == 1? cosValue = 2*PI : cosValue = PI;
        }

        //calculate new value
        if(cosValue > 3*PI) cosValue = PI;
        cosValue += ((millis() - lastFlash)/(float)fadeTime) * PI;
        ledValue = ((cos(cosValue) + 1) * 255) / 2;

        lastFlash = millis();
    }

    // Type between 100 and 200 -> direct value
    if (type >= 100 && type <= 200) { 
        ledValue = (type - 100)*2.55;
    }

    if(lastLedValue != ledValue) { 
        ledcWrite(0, ledValue);
        lastLedValue = ledValue;
    }

    lastType = type;
}
