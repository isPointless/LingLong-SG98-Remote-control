#include "gbw.h"
#include "AcaiaArduinoBLE.h"
#include "main.h"
#include "io.h"
#include "pdata.h"

#ifdef JMC_DRIVE
#include "motorcontrol_jmc.h"
#endif

#ifdef RT_DRIVE
#include "motorcontrol_rt.h"
#endif

#define DEBUG_GBW
//#define DEBUG_FAKE_SCALE

AcaiaArduinoBLE scale(false);

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

GBW_WEIGHT _shot[300];
uint16_t last_shot_updated;


//BLEService weightService("0x0FFE"); // create service
//BLEByteCharacteristic weightCharacteristic("0xFF11",  BLEWrite | BLERead);

//SEPARATE TASK!!
void scales_init() { 
    BLE.setLocalName("GbW-Service");
    // initialize the Bluetooth® Low Energy hardware
    BLE.begin();
    // Optionally add your Mac Address as an argument: acaia.init("##:##:##:##:##:##");
    if(scale.init()) {
        scale.tare();
        scale.tare();
    }
}

//SEPARATE TASK!!
void gbwVitals() 
{   
    static uint8_t prevScaleStatus;
    static unsigned long lastConnected;
    static int32_t prevWeight;
    // Send a heartbeat message to the scale periodically to maintain connection
    if(scale.heartbeatRequired() && scale.isConnected())
    {
        scale.heartbeat();
    }
    // always call newWeightAvailable to actually receive the datapoint from the scale,
    // otherwise getWeight() will return stale data
    if(scale.newWeightAvailable())
    {   
        #ifndef DEBUG_FAKE_SCALE
            currentWeight = scale.getWeight() * 1000; // we only deal in mg
        #endif

        #ifdef DEBUG_SCALE
            Serial.println(float(float(currentWeight)/1000));
        #endif

        #ifdef DEBUG_FAKE_SCALE
            currentWeight = fakeScale();
        #endif

        newWeightAvail = true;
    }

    // Manage connection states 
    if(state == IDLE_GBW || scale._isScanning == true || scale._isConnecting == true) {
        if(scale.isConnected() == false) scale.init();
    } else {
        if(state == IDLE || state == GRINDING || state == SLEEPING)  {
            scale.disconnect();
            scale.stopScan();
        }
    }

    if(scale.isConnected() == true) {
        scaleStatus = SCALE_CONNECTED;
        lastConnected = millis();
        scale.stopScan();
    }

    if(scale.isConnected() == false) {
        scaleStatus = SCALE_DISCONNECTED; 
        if(lastConnected + 5000 > millis() && millis() > 5000) scaleStatus = SCALE_LOST;
        if(scale._isConnecting == true) scaleStatus = SCALE_CONNECTING;
        currentWeight = 0;
    }

    if(scaleStatus != prevScaleStatus) { 
        newData = true;
        prevScaleStatus = scaleStatus;
    }

    if(prevWeight != currentWeight) { 
        newData = true;
        prevWeight = currentWeight;
    }

}

void disconnect_for_sleep() { 
    scale.stopScan();
    scale.disconnect();
}

int16_t fakeScale() { 
    //TEST SCALE DATA
    static int16_t lastWeight;
    if(gbw_started == true && grindingComplete == false) {
        currentWeight = (millis() - startOfShot) * 3; 
        
        static int16_t lastWeight;
        if(currentWeight != lastWeight) { 
            newData = true;
        }
        Serial.print("  Last shot updated: ");
        Serial.println(last_shot_updated);
        lastWeight = currentWeight;
    }

    //if(grindingComplete == true) return lastWeight;

    return currentWeight;
}

void do_gbw() { 
    static unsigned long lastCall;
    static unsigned long lastUpdate;
    static unsigned long tareTime;
   // gbwVitals();
   // fakeScale();


    //reset route 
    if(lastCall + 200 < millis()) { //reset after last call 200ms ago
        if(motor_setRPM != 0) motorOff();
        #ifdef DEBUG_GBW
            currentWeight = 0;
        #endif

        for(int i = 0; i < sizeof(_shot)/sizeof(_shot[0]); i++) { 
            _shot[i].weight = 0;
            _shot[i].time = 0;
        }

        if(scaleStatus != SCALE_CONNECTED) { 
            state = IDLE_GBW;
            error = 106;
            newData = true;
            return;
        }
        //check tare
        if(abs(currentWeight) > 100) { // we accept 200mg of deviation 
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
            #ifdef DEBUG_GBW
                Serial.println("gbw started!");
            #endif
            newData = true;
        }
        slow_phase = false;
        slow_phase_at = 0;

        lastUpdate = millis();
    }

    //Tare route
    if(gbw_started == false && tareTime + 2000 < millis()) {
        if(currentWeight < 200) { 
        startOfShot = millis();
        last_shot_updated = 0;
        grindingComplete = false;
        gbw_started = true;
        newData = true;
        #ifdef DEBUG_GBW
            Serial.println("gbw started!");
        #endif
        lastUpdate = millis();
        } else { 
            scale.tare();
            tareTime = millis();
            gbw_started = false;
            #ifdef DEBUG_GBW
                Serial.println("Tare");
            #endif
        }
    }

    //Actually grinding! Menu5 = button swing delay
    if(gbw_started == true && startOfShot + Menu3[5].value < millis() && grindingComplete == false && slow_phase == false) { 
        motor_setRPM = Menu3[1].value; //This is the GBW set RPM in menu
    }

    // Switch to slow phase if applicable
    if(gbw_predict() < Menu3[2].value && slow_phase == false && grindingComplete == false && gbw_started == true) { 
        motor_setRPM = Menu3[3].value;
        slow_phase = true;
        slow_phase_at = millis() - startOfShot; 
        #ifdef DEBUG_GBW
            Serial.println("slow phase..");
        #endif
    }

    //Completion route (menu 6 = gbw offset time)
    if(gbw_predict() < Menu3[6].value && grindingComplete == false && gbw_started == true) { 
        grindingComplete = true;
        //motorOff();        
        lastUpdate = millis();
        #ifdef DEBUG_GBW
            Serial.println("Grinding complete!");
        #endif
        newData = true;
    }

    //Scale issues route
    if(scale.isConnected() == false && grindingComplete == false) { 
        #ifdef DEBUG_GBW
            Serial.println("Scale issues..");
        #endif
        motorOff();
        grindingComplete = true;
        gbw_started = false;
        state = IDLE_GBW;
        error = 104;
        newData = true;
    } 

    //No grounds coming out after 10s, no beans?
    if(startOfShot + 10000 < millis() && grindingComplete == false && gbw_started == true && abs(currentWeight) < 200) { 
        motorOff();
        grindingComplete = false;
        gbw_started = false;
        state = IDLE_GBW;
        error = 105;
        newData = true;
    }
  
    // Finishing and learn route
    if(grindingComplete == true && lastUpdate + 500 > millis()) { 
        if(motor_setRPM != 0) motorOff();
        ledAction(1);
        motor_setRPM = 0;
    }

    // exit route
    if(grindingComplete == true && lastUpdate + 500 < millis()) { 
        if(motor_setRPM != 0) motorOff();
        #ifdef DEBUG_GBW
            Serial.println("learn!");
        #endif
        gbw_learn(); //only needs to run once
        state = IDLE_GBW;

        #ifdef DEBUG_GBW
            Serial.println("return from learn..!");
        #endif
        lastUpdate = millis();
        lastActivity = millis();
        newData = true;
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

    lastCall = millis();
}


int16_t gbw_predict() {
    int16_t predictedTime = 5000;
    static int16_t lastPredictedTime;
    speedModifier = Menu3[4].value / 10000.0f;

    if(state == GRINDING_GBW && gbw_started == true) {  
        #ifdef DEBUG_FAKE_SCALE
        float rpm = Menu3[1].value;
        #else
        float rpm = slow_phase ? Menu3[3].value : Menu3[1].value;
        #endif
        float denom = speedModifier * (rpm / 60.f);
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

    return predictedTime;
}


void gbw_learn() { 
    int overshoot;
    if(abs((int32_t)setWeight - currentWeight) < 50) { //accept some deviation
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
        if(_shot[i].weight > 3000 && firstVal == 0) {
            firstVal = i;
        }
        if(_shot[i].weight > 10000 && secondVal == 0) {
            secondVal = i;
        }
        if(firstVal != 0 && secondVal != 0) {
            break;
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

    float rpm = float(Menu3[1].value);
    if (rpm == 0) return; // Prevent division by zero

    float ground_per_rotation = grindSpeed / (rpm / 60.f); // g/s / rot/s = g / rot
    
    if(speedModifier != 1) speedModifier = (ground_per_rotation + speedModifier)/2;
    else speedModifier = ground_per_rotation;

    //If the speedmodifier is quite accurate ie within 5%, change the pre stop value.
    if(Menu3[4].value < (speedModifier*10500) && Menu3[4].value > (speedModifier * 9500)) {
        overshoot = currentWeight - setWeight;
        if(abs(overshoot) > 50) { 
            if(Menu3[6].value == default_time_offset) Menu3[6].value = Menu3[6].value + overshoot/2;
            else Menu3[6].value = Menu3[6].value + overshoot/8;
        }
        #ifdef DEBUG_GBW
            Serial.print("new overshoot! Last weight: ");
            Serial.print(_shot[last_shot_updated - 1].weight);
            Serial.print(" overshoot value: ");
            Serial.print(overshoot);
        #endif
    }

    #ifdef DEBUG_GBW
        Serial.print("currentWeight: ");
        Serial.print(currentWeight);
        Serial.print(" ground per rot: ");
        Serial.print(ground_per_rotation);
        Serial.print(" overshoot: ");
        Serial.println(Menu3[6].value);
    #endif
    
    Menu3[4].value = speedModifier*10000; //always save the new value.
    pdata_write(4);
}

