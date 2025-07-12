#include "gbw.h"
#include "main.h"
#include "io.h"
#include "pdata.h"
#include "display.h"

#ifdef JMC_DRIVE
#include "motorcontrol_jmc.h"
#endif

#ifdef RT_DRIVE
#include "motorcontrol_rt.h"
#endif

unsigned long startOfShot = 0;
float speedModifier = 1;
bool slow_phase = false;

//shared
volatile uint16_t slow_phase_at = 0;
volatile bool grindingComplete = false;
volatile int16_t newWeightAvail;
volatile int32_t currentWeight = 0; //in MG
volatile bool scaleConnected = false;
volatile SCALE_STATUS scaleStatus = INVALID_SCALE_STATUS;
volatile bool gbw_started = false;
volatile bool saveScaleEnabled = false;
bool connect = false;
bool maintain = true;

// MUTEX >>
SemaphoreHandle_t scaleMutex;
String scale_connect_mac = "";
String scale_connect_name = "";
String scale_mac = "";
String scale_name = "";
// MUTEX <<

String mac_local = "";

#define DEBUG true
BLEScale scale(DEBUG);

GBW_WEIGHT _shot[300];
uint16_t last_shot_updated;

//BLEService weightService("0x0FFE"); // create service
//BLEByteCharacteristic weightCharacteristic("0xFF11",  BLEWrite | BLERead);

//SEPARATE TASK!!
void scales_init() { 
    NimBLEDevice::init("GbWService");
    if (xSemaphoreTake(scaleMutex, portMAX_DELAY)) {
        mac_local = scale_connect_mac;
        xSemaphoreGive(scaleMutex);
    } 
}

//SEPARATE TASK!!
void gbwVitals() 
{   
    static uint8_t prevScaleStatus;
    static unsigned long lastConnected;
    static int32_t prevWeight;

    static unsigned long lastTare ;

    if(scale.manage(connect, maintain, mac_local) == true) { 
        scaleStatus = SCALE_CONNECTED;
        if (xSemaphoreTake(scaleMutex, portMAX_DELAY)) {
            scale_mac = scale.connected_mac;
            scale_name = scale.connected_name;
            xSemaphoreGive(scaleMutex);
        } 
    }

    if(scale.isConnected() == true) { 
        scaleStatus = SCALE_CONNECTED;
        if(lastTare + 2000 < millis()) { 
            scale.tare();
            scale.startTimer();
            lastTare = millis();
        }
        
    } else { 
        if(scale._isConnecting == true) scaleStatus = SCALE_CONNECTING;
        else { 
            scaleStatus = SCALE_DISCONNECTED;
            currentWeight = 0;
        }
    }

    static unsigned long lastCheck;
    if(lastCheck + 5000 < millis()) { 
        lastCheck = millis();

        if (xSemaphoreTake(scaleMutex, portMAX_DELAY)) {
            mac_local = scale_connect_mac;
            xSemaphoreGive(scaleMutex);
        }
    }

    // always call newWeightAvailable to actually receive the datapoint from the scale,
    // otherwise getWeight() will return stale data
    if(scale.newWeightAvailable())
    {   
        currentWeight = scale.getWeight() * 1000; // we only deal in mg
        newWeightAvail = true;
        disp_updateRequired = true;
    }

    // Manage connection states 
    if(state == IDLE_GBW || state == GRINDING_GBW) {
        connect = true;
        maintain = true;
    } else {
        if(state == IDLE || state == GRINDING || state == SLEEPING)  {
            maintain = false;
        }
        connect = false;
    }

    if(scaleStatus != prevScaleStatus) { 
        disp_updateRequired = true;
        prevScaleStatus = scaleStatus;
    }

    if(prevWeight != currentWeight) { 
        disp_updateRequired = true;
        prevWeight = currentWeight;
    }

}

void disconnect_for_sleep() { 
    maintain = false;
    connect = false;
    scale.disconnect();
}

void do_gbw() { 
    static unsigned long lastCall;
    static unsigned long lastUpdate;
    static unsigned long tareTime;

    if(commStatus != CONNECTED) { 
        motor_setRPM = 0;
        state = IDLE_GBW;
    }

    //reset route 
    if(lastCall + 200 < millis()) { //reset after last call 200ms ago
        if(motor_setRPM != 0) motor_setRPM = 0;
        gbw_started = false;
        grindingComplete = false;

        for(int i = 0; i < sizeof(_shot)/sizeof(_shot[0]); i++) { 
            _shot[i].weight = 0;
            _shot[i].time = 0;
        }

        if(scaleStatus != SCALE_CONNECTED) { 
            state = IDLE_GBW;
            error = 106;
            disp_updateRequired = true;
            Serial.println("Scale not connected!"); 
            return;
        }
        //check tare
        if(abs(currentWeight) > 100) { // we accept 100mg of deviation 
            scale.tare();
            tareTime = millis();
            gbw_started = false;
            #ifdef DEBUG_GBW
                Serial.println("Tare");
            #endif

        } else { //no tare needed
            startOfShot = millis();
            last_shot_updated = 0;
            grindingComplete = false;
            gbw_started = true; //coffee imminent
            tareTime = millis(); //for safety?
            #ifdef DEBUG_GBW
                Serial.println("gbw started!");
            #endif
            disp_updateRequired = true;
        }
        slow_phase = false;
        slow_phase_at = 0;
        lastUpdate = millis();
    }
    lastCall = millis();

    //Tare route
    if(gbw_started == false && tareTime + 2000 < millis()) {
        if(currentWeight < 200) { 
            startOfShot = millis();
            last_shot_updated = 0;
            grindingComplete = false;
            gbw_started = true;
            disp_updateRequired = true;
            #ifdef DEBUG_GBW
                Serial.println("gbw started!");
            #endif
            lastUpdate = millis();
        } else { //TARE AGAIN IF NEEDED
            scale.tare();
            tareTime = millis();
            gbw_started = false;
            #ifdef DEBUG_GBW
                Serial.println("Tare");
            #endif
        }
    }

    //Actually grinding! Menu5 = button swing delay
    if(gbw_started == true && startOfShot + Menu3[GBW_BUTTON_DELAY].value < millis() && grindingComplete == false && slow_phase == false) { 
        motor_setRPM = Menu3[GBW_RPM_SET].value; //This is the GBW set RPM in menu
    }

    if(Menu3[GBW_SLOW_PHASE].value > 0) {
        // Switch to slow phase if applicable
        if(gbw_predict() < (Menu3[GBW_SLOW_PHASE].value) && slow_phase == false && grindingComplete == false && gbw_started == true) { 
            motor_setRPM = Menu3[GBW_SLOW_RPM].value;
            slow_phase = true;
            slow_phase_at = millis() - startOfShot; 
            #ifdef DEBUG_GBW
                Serial.println("slow phase..");
            #endif
        }
    }

    //Completion route
    if(gbw_predict() < 0 && grindingComplete == false && gbw_started == true) { 
        grindingComplete = true;
        motor_setRPM = 0;
        lastUpdate = millis();
        #ifdef DEBUG_GBW
            Serial.println("Grinding complete!");
        #endif
        disp_updateRequired = true;
    }

    //Scale issues route
    if(scale.isConnected() == false && grindingComplete == false) { 
        #ifdef DEBUG_GBW
            Serial.println("Scale issues..");
        #endif
        motor_setRPM = 0;
        grindingComplete = true;
        gbw_started = false;
        state = IDLE_GBW;
        error = 104;
        disp_updateRequired = true;
    } 

    //No grounds coming out after 10s, no beans?
    if(startOfShot + 10000 < millis() && grindingComplete == false && gbw_started == true && abs(currentWeight) < 200) { 
        motor_setRPM = 0;
        grindingComplete = false;
        gbw_started = false;
        state = IDLE_GBW;
        error = 105;
        disp_updateRequired = true;
    }
  
    // Finishing and learn route
    if(grindingComplete == true && lastUpdate + 500 > millis()) { 
        if(motor_setRPM != 0) motorOff();
        ledAction(1);
        motor_setRPM = 0;
    }

    // exit route
    if(grindingComplete == true && lastUpdate + 500 < millis()) { 
        scale.startTimer(); //BEEP!
        delay(25);
        scale.stopTimer(); 
        if(motor_setRPM != 0) motorOff();
        #ifdef DEBUG_GBW
            Serial.println("learn!");
        #endif
        gbw_learn(); //only needs to run once
        #ifdef DEBUG_GBW
            Serial.println("return from learn..!");
        #endif
        lastUpdate = millis();
        lastActivity = millis();
    }

    if(grindingComplete == true && lastUpdate + 2000 < millis()) { 
        state = IDLE_GBW;
        disp_updateRequired = true;
    }

    //keep track of our shot
    if(state == GRINDING_GBW && gbw_started == true && grindingComplete == false && newWeightAvail == true) { 
        _shot[last_shot_updated].weight = abs(currentWeight);
        _shot[last_shot_updated].time = millis() - startOfShot;
        last_shot_updated++;
        if(last_shot_updated >= sizeof(_shot)/sizeof(_shot[0])) last_shot_updated = 0;
        newWeightAvail = false;
    }

    // flash accordingly
    if(gbw_started == false) ledAction(2);
    if(gbw_started == true && grindingComplete == false) ledAction(100 * (currentWeight / setWeight));
    if(grindingComplete == true) ledAction(1);
}


int16_t gbw_predict() {
    int16_t predictedTime = 5000;
    static int16_t lastPredictedTime;
    speedModifier = Menu3[GBW_SPEEDMOD].value / 10000.0f;

    if(state == GRINDING_GBW && gbw_started == true) {  
        int16_t rpm = slow_phase? Menu3[GBW_SLOW_RPM].value : Menu3[GBW_RPM_SET].value;

        float denom = speedModifier * (rpm / 60.f); //RPM /60 = rot/s -> rot/s * grams / rot 
        if (denom != 0) {
            float stillToGrind = setWeight - currentWeight; 
            predictedTime = stillToGrind / denom;
        } else {
            predictedTime = 0;
        }
    }
    

    if(gbw_started == true && grindingComplete == true) predictedTime = 0;

    #ifdef DEBUG_GBW
        if(lastPredictedTime != predictedTime) { 
            Serial.println(predictedTime);
            lastPredictedTime = predictedTime;
        }
    #endif

    return predictedTime - Menu3[GBW_OFFSET].value;
}


void gbw_learn() { 
    int overshoot;
    if(abs((int32_t)setWeight - currentWeight) < 100) { //accept some deviation
        return;
    }  

    uint16_t firstVal = 0;
    uint16_t secondVal = 0;

    #ifdef DEBUG_GBW
        for(int i = 0; i < last_shot_updated; i++) { 
            Serial.print("shotv: ");
            Serial.print(i);
            Serial.print("  weight: ");
            Serial.print(_shot[i].weight);
            Serial.print(" time: ");
            Serial.println(_shot[i].time);
        }
    #endif
    
    // Lookup the grinding speed 
    for(int i = 0; i < last_shot_updated; ++i) {
        if(_shot[i].weight > 2000 && firstVal == 0) {
            firstVal = i;
        }
        if(_shot[i].weight > 10000 && secondVal == 0) {
            secondVal = i;
        }
        if(firstVal != 0 && secondVal != 0) {
            break;
        }
    }

    //we switched to slow mode before test weight of 10g was reached.
    if(slow_phase_at < _shot[secondVal].time) {  
        secondVal = 0;
        for(int i = last_shot_updated; i < 1; i--) { 
            if(_shot[i].time < slow_phase_at && secondVal == 0) {
            secondVal = i;
            }
            if(secondVal != 0) {
            break;
            }
        }
    }

    int16_t test_weight_diff = _shot[secondVal].weight - _shot[firstVal].weight;
    uint32_t test_time_diff = _shot[secondVal].time - _shot[firstVal].time;

    #ifdef DEBUG_GBW
        Serial.print("Val1: ");
        Serial.print(firstVal);
        Serial.print("  Val2: ");
        Serial.print(secondVal);
        Serial.print(" test_time_diff: ");
        Serial.print(test_time_diff);
        Serial.print(" last shot updated: ");
        Serial.println(last_shot_updated);
    #endif

    if (test_time_diff == 0) return; // Prevent division by zero
    if(firstVal == 0 || secondVal == 0) return;

    float grindSpeed = test_weight_diff / float(test_time_diff);

    float rpm = float(Menu3[GBW_RPM_SET].value);
    if (rpm == 0) return; // Prevent division by zero

    float ground_per_rotation = grindSpeed / (rpm / 60.f); // g/s / rot/s = g / rot
    
    if(speedModifier != 0.5) speedModifier = (ground_per_rotation + speedModifier)/2;
    else speedModifier = ground_per_rotation;

    //If the speedmodifier is quite accurate ie within 5%, change the pre stop value.

    overshoot = currentWeight - setWeight;
    if(abs(overshoot) > 100) { 
        if(Menu3[GBW_OFFSET].value == default_time_offset) Menu3[GBW_OFFSET].value = Menu3[GBW_OFFSET].value + overshoot/5;
        else Menu3[GBW_OFFSET].value = Menu3[GBW_OFFSET].value + overshoot/12;
    } 
    #ifdef DEBUG_GBW
        Serial.print("new overshoot! Last weight: ");
        Serial.print(_shot[last_shot_updated - 1].weight);
        Serial.print(" overshoot value: ");
        Serial.print(overshoot);
    #endif
    

    #ifdef DEBUG_GBW
        Serial.print("currentWeight: ");
        Serial.print(currentWeight);
        Serial.print(" ground per rot: ");
        Serial.print(ground_per_rotation);
        Serial.print(" overshoot: ");
        Serial.println(Menu3[GBW_OFFSET].value);
    #endif
    
    Menu3[GBW_SPEEDMOD].value = speedModifier*10000; //always save the new value.
    pdata_write(4);
}

