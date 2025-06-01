#include "definitions.h"
#include "io.h"
#include "main.h"
#include "motorcontrol.h"
#include <math.h>
#include <JC_Button.h>
#include <ESP32Encoder.h>


Button enc_button(ENC_BTN);
Button start_button(BTN);
ESP32Encoder encoder;

bool longPress = false;
int32_t v = 0;
int32_t lastV = 0;

void io_init() { 
    pinMode(BTN, INPUT);
    pinMode(ENC_BTN, INPUT);
    pinMode(start_button_led, OUTPUT);

    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);
    
    encoder.attachHalfQuad(ENC_A, ENC_B);
    encoder.setFilter(1023);
    encoder.setCount(0);
}

void do_io() { 
    if(state == IDLE) { 
        setRPM =+ encoder_change()*rpm_scalar;
        
        if(START_BUTTON() == true) state = GRINDING;
        if(ENC_BUTTON() == true) { 
            if(longPress == false) { 
                state = MENU1;
            } else longPress = false;
        }
        if(pressedLong() == true) { 
            longPress = true;
        } 

        if(longPress == true && ENC_BUTTON() == false) { 
            state = IDLE_GBW;
        }
    } 

    if(state == IDLE_GBW) { 
        setWeight =+ encoder_change();
        
        if(START_BUTTON() == true && longPress == false) state = GRINDING_GBW;
        if(ENC_BUTTON() == true) { 
            if(longPress == false) { 
                state = MENU1;
            } else longPress = false;
        }
        //longpress switches to IDLE <--> IDLE_GBW
        if(pressedLong() == true) { 
            longPress = true;
        } 

        if(pressedLong() == true) { 
            longPress = true;
        }

        if(longPress == true && ENC_BUTTON() == false) { 
            state = IDLE_GBW;
        }
        
    }

    if(state == GRINDING) { 
        setRPM =+ encoder_change()*rpm_scalar;

        if(START_BUTTON() == true) { 
            state = (purge_enabled? PURGING : IDLE);
        }
        if(ENC_BUTTON() == true) { 
            state = IDLE;
            motor_setRPM = 0;
        }
    }

    if(state == PURGING) { 
        setRPM =+ encoder_change()*rpm_scalar;

        if(START_BUTTON() == true) { 
            state = IDLE;
            motor_setRPM = 0;
        }
        if(ENC_BUTTON() == true) { 
            state = IDLE;
            motor_setRPM = 0;
        }
    }

    if(state == GRINDING_GBW) { 
        //setWeight =+ encoder_change();

        if(START_BUTTON() == true) { 
            state = IDLE_GBW;
            motor_setRPM = 0;
        }
        if(ENC_BUTTON() == true) { 
            state = IDLE_GBW;
            motor_setRPM = 0;
        }
    }

    if(state == MENU1) { 
        menu1Selected =+ encoder_change();
        if(menu1Selected > 6) menu1Selected = 0;
        if(menu1Selected < 0) menu1Selected = 6;

        if(START_BUTTON() == true) { 
            // do.. nothing?
        }
        if(enterMenu == true) 

        if(ENC_BUTTON() == true) { 
            if(menu1Selected == 2) { 
                state = MENU2;
            } 
        }
    }

}

bool pressedLong() { 
    if(enc_button.pressedFor(LONGPRESS)) return true;
    return false;
}

int16_t encoder_change() { 
    int16_t change;
    v = encoder.getCount()/ENC_TOL; 

    change = lastV - v;
    lastV = v;

    if(change != 0) lastActivity = millis();
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
        return true;
    }
    return false;
}

bool START_BUTTON() {
    start_button.read();
    if (start_button.wasReleased()) { 
        lastActivity = millis();
        return true;
    }
    return false;
}

void ledAction(uint8_t type) {  //Type: 0 = off, 1 = on, 2 = flashing, 3 = fading, 4 flashing fast, 5 = flashing fastest
    static uint8_t lastType = 0;
    static unsigned long lastFlash = millis();

    if (type == 0) { 
        analogWrite(start_button_led, 0);
    }

    if (type == 1) {
        analogWrite(start_button_led, 255);
    }

    //flashing
    if (type == 2 || type == 4 || type == 5) { 
        static bool ledOn = true;

        if(lastType != type) 
        { 
            ledOn = true;
            analogWrite(start_button_led, 255);
            lastFlash = millis();
        }


        if(lastFlash + (type == 2? flashTime : type == 4? fastFlashTime : fastestFlashTime) < millis()) 
        { 
            ledOn = !ledOn;
            analogWrite(start_button_led, ledOn == true? 255 : 0);
            lastFlash = millis();
        }
    }

    if (type == 3) { //fading
    static double cosValue = 0;
    static uint8_t ledValue = 0;
    static uint8_t lastLedValue = 0;

    if (lastType != type) { 
        lastType == 1? cosValue = 2*PI : cosValue = PI;
        lastFlash = millis();
    }

    cosValue > 3*PI? cosValue = PI : 0;
    cosValue += ((millis() - lastFlash)/(float)fadeTime) * PI;
    ledValue = ((cos(cosValue) + 1) * 255) / 2;

    lastFlash = millis();

    if(lastLedValue != ledValue) { 
        analogWrite(start_button_led, ledValue);
        lastLedValue = ledValue;
    }
    }

    lastType = type;
}
