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

//shared between tasks!
volatile int32_t currentWeight = 0; //in MG
volatile bool scaleConnected = false;
volatile SCALE_STATUS scaleStatus = INVALID_SCALE_STATUS;
volatile int16_t newWeightAvail;
//Only used in scaletask:
bool connect = false;
bool maintain = true;

//Used in main loop:
uint16_t slow_phase_at = 0;
bool grindingComplete = false;
bool gbw_started = false;
unsigned long shotStopped = 0;


// MUTEX >>
SemaphoreHandle_t scaleMutex;
String scale_connect_mac = "";
String scale_connect_name = "";
String scale_mac = "";
String scale_name = "";
// MUTEX <<

String mac_local = "";

BLEScale scale(DEBUG_SCALE);

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
    } else newWeightAvail = false; //Here coz thats one round!

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
    static bool properShot;
    static bool learned;

    #ifndef DEBUG_GBW
    if(commStatus != CONNECTED) { 
        motorOff();
        state = IDLE_GBW;
    }
    #endif

    //reset route & start route
    if(lastCall + 1000 < millis()) { //reset after last call 1000ms ago
        if(motor_setRPM != 0) motor_setRPM = 0;
        //Set starting values properly
        gbw_started = false;
        grindingComplete = false;
        slow_phase = false;
        startOfShot = 0;
        last_shot_updated = 0;
        slow_phase_at = 0;
        lastUpdate = millis();
        shotStopped = 0;
        properShot = false;
        learned = false;

        //Zero shot data
        for(int i = 0; i < sizeof(_shot)/sizeof(_shot[0]); i++) { 
            _shot[i].weight = 0;
            _shot[i].time = 0;
        }

        //Get outta here if scale is not connected;
        if(scaleStatus != SCALE_CONNECTED) { 
            state = IDLE_GBW;
            error = 106;
            disp_updateRequired = true;
            #ifdef DEBUG_GBW
                Serial.println("Scale not connected!"); 
            #endif
            return;
        }

        //check tare
        if(abs(currentWeight) > 100) { // we accept 100mg of deviation.. this is natural for the scale..
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
    }

    lastCall = millis();

    //Tare route
    if(gbw_started == false && tareTime + 2000 < millis()) {
        if(abs(currentWeight) < 200) { 
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
        shotStopped = millis() - startOfShot;
        #ifdef DEBUG_GBW
            Serial.println("Grinding complete!");
        #endif
        disp_updateRequired = true;
        properShot = true;
    }

    //Backup completion route
    if(currentWeight > setWeight + 1000 && motor_setRPM > 0) { 
        grindingComplete = true;
        motor_setRPM = 0;
        lastUpdate = millis();
        shotStopped = millis() - startOfShot;
        #ifdef DEBUG_GBW
            Serial.println("Backup completion triggered.. bad!");
            Serial.println(currentWeight);
            Serial.println(setWeight);
        #endif
        properShot = false;
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
        properShot = false;
    } 

    //No grounds coming out after 3s, no beans?
    if(startOfShot + 3000 < millis() && grindingComplete == false && gbw_started == true && abs(currentWeight) < 200) { 
        motor_setRPM = 0;
        grindingComplete = false;
        gbw_started = false;
        slow_phase = false;
        properShot = false;
        state = IDLE_GBW;
        error = 105;
        disp_updateRequired = true;
    }

    // Learn route
    if(grindingComplete == true && lastUpdate + 600 < millis() && learned == false) { 
        scale.startTimer(); //BEEP!
        delay(25);
        scale.stopTimer(); 
        if(motor_setRPM != 0) motorOff();
        #ifdef DEBUG_GBW
            Serial.println("learn!");
        #endif
        if(properShot) gbw_learn(); //only needs to run once
        #ifdef DEBUG_GBW
            Serial.println("return from learn..!");
        #endif
        learned = true;
        lastActivity = millis();
    }

    if(grindingComplete == true && lastUpdate + 2000 < millis()) { 
        state = IDLE_GBW;
        disp_updateRequired = true;
        grindingComplete = false;
        gbw_started = false;
        tareTime = 0;
        slow_phase = false;
        slow_phase_at = 0;
        properShot = false;
    }

    //keep track of our shot
    if(grindingComplete == false) {
        if(state == GRINDING_GBW && gbw_started == true && newWeightAvail == true ) { 
            _shot[last_shot_updated].weight = abs(currentWeight);
            _shot[last_shot_updated].time = millis() - startOfShot;
            last_shot_updated++;
            if(last_shot_updated >= sizeof(_shot)/sizeof(_shot[0])) last_shot_updated = 0;
            newWeightAvail = false;
        }
    } else { 
        if(lastUpdate + 600 > millis()) { 
            if(state == GRINDING_GBW && gbw_started == true && newWeightAvail == true ) { 
                _shot[last_shot_updated].weight = abs(currentWeight);
                _shot[last_shot_updated].time = millis() - startOfShot;
                last_shot_updated++;
                if(last_shot_updated >= sizeof(_shot)/sizeof(_shot[0])) last_shot_updated = 0;
                newWeightAvail = false;
            }
        }
    }

    // flash accordingly
    if(gbw_started == false) ledAction(0);
    if(gbw_started == true && grindingComplete == false) ledAction(100 + 100*(currentWeight / setWeight));
    if(grindingComplete == true) ledAction(1);
}


int16_t gbw_predict() {
    int16_t predictedTime = 5000;
    static unsigned long lastCall;
    static int16_t lastActualPrediction;
    static unsigned long lastPredictionAt;
    speedModifier = Menu3[GBW_SPEEDMOD].value / 10000.0f;

    if(lastCall + 1000 < millis()) { 
        lastActualPrediction = 5000;
        predictedTime = 5000;
        newWeightAvail = true;
    }

    lastCall = millis();

    if(state == GRINDING_GBW && gbw_started == true) {  
        int16_t rpm = slow_phase? Menu3[GBW_SLOW_RPM].value : Menu3[GBW_RPM_SET].value;

        float denom = speedModifier * (rpm / 60.f); //RPM /60 = rot/s -> rot/s * grams / rot 
        if (denom != 0) {
            float stillToGrind = setWeight - currentWeight; 
            predictedTime = stillToGrind / denom;
        } else {
            predictedTime = 0;
        }

        if(predictedTime != lastActualPrediction) { 
            lastPredictionAt = millis();
            lastActualPrediction = predictedTime;
            #ifdef DEBUG_GBW
                Serial.print("New prediction: "), Serial.print(predictedTime), Serial.print(" ..at: "), Serial.println(lastPredictionAt - startOfShot);
            #endif
        }
    }
    predictedTime = lastActualPrediction - (millis() - lastPredictionAt); //Give a continuous prediction value

    if(gbw_started == true && grindingComplete == true) predictedTime = 0;
    return predictedTime - abs(Menu3[GBW_OFFSET].value);
}


void gbw_learn() { 
    int overshoot;
    bool isProper = false;
    int16_t newOffset;
    uint16_t firstVal = 0;
    uint16_t secondVal = 0;

    if(abs((int32_t)setWeight - currentWeight) > 5000) isProper = false;
    else isProper = true;

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
    
    // Lookup the grinding speed with time between 2 and 10g
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

    if (test_time_diff < 1000) return; // It's not a proper shot.
    if(firstVal == 0 || secondVal == 0) return; //not a proper shot
    if (test_time_diff == 0) return; // Prevent division by zero

    //Calculations
    float grindSpeed = test_weight_diff / float(test_time_diff);

    float rpm = Menu3[GBW_RPM_SET].value;
    if (rpm == 0) return; // Prevent division by zero

    float ground_per_rotation = grindSpeed / (rpm / 60.f); // g/s / rot/s = g * rot
    
    if(speedModifier != 0.5) speedModifier = (ground_per_rotation + speedModifier)/2;
    else speedModifier = ground_per_rotation;

    #ifdef DEBUG_GBW
        Serial.print("old speedmod: ");
        Serial.print(Menu3[GBW_SPEEDMOD].value);
        Serial.print("  New speedmod: ");
        Serial.println(speedModifier*10000);
    #endif

    //Calculate offsets
    overshoot = currentWeight - setWeight;

    if(Menu3[GBW_OFFSET].value == default_time_offset) newOffset = Menu3[GBW_OFFSET].value + overshoot/5;
    else newOffset = Menu3[GBW_OFFSET].value + overshoot/12;
    if(newOffset < 100 || newOffset > 600) isProper = false; // Not a proper shot
    else isProper = true;

    #ifdef DEBUG_GBW
        Serial.print("lastOffset: "), Serial.print(Menu3[GBW_OFFSET].value), Serial.print(" Calculated offset: "), Serial.println(newOffset);
    #endif

    Serial.print(" Proper?" ), Serial.println(isProper);

    if(isProper) { 
        Menu3[GBW_SPEEDMOD].value = speedModifier*10000; //always save the new value.
        Menu3[GBW_OFFSET].value = newOffset;
        pdata_write(4);
    }
}

