#include "gbw.h"
#include <AcaiaArduinoBLE.h>
#include "main.h"

#ifdef JMC_DRIVE
#include "motorcontrol_jmc.h"
#endif

#ifdef RT_DRIVE
#incldue "motorcontrol_rt.h"
#endif

AcaiaArduinoBLE scale(DEBUG);

uint16_t goalWeight = 1800; //weight in g * 100
bool scaleConnected = false;
float speedModifier;

BLEService weightService("0x0FFE"); // create service
BLEByteCharacteristic weightCharacteristic("0xFF11",  BLEWrite | BLERead);


float currentWeight = 0;

void scales_init() { 
  BLE.begin();
  BLE.setLocalName("SG-GbW");
  BLE.setAdvertisedService(weightService);
  weightService.addCharacteristic(weightCharacteristic);
  BLE.addService(weightService);
  weightCharacteristic.writeValue(goalWeight);
  BLE.advertise();
  #ifdef DEBUG
  Serial.println("Bluetooth® device active, waiting for connections...");
  #endif
}

bool scale_connected() { 
    if(scaleConnected == true)  return true; 
    else return false;
}

void gbwVitals() 
{   
    BLE.poll();
    // Send a heartbeat message to the scale periodically to maintain connection
    if(scale.heartbeatRequired())
    {
        scale.heartbeat();
    }

    // always call newWeightAvailable to actually receive the datapoint from the scale,
    // otherwise getWeight() will return stale data
    if(scale.newWeightAvailable())
    {
        currentWeight = scale.getWeight();
    }

    if(scale.isConnected() == true) scaleConnected = true; else scaleConnected = false;

    #ifdef DEBUG
        Serial.print(currentWeight);
    #endif
}

uint16_t gbw_predict() {  //returned value is the predicted time in ms till the motor has to stop
    static float predictedTime;
    static float stillToGrind;
    if(state == IDLE_GBW) { 
        predictedTime = goalWeight / ((float)setRPM/1000) + 100; //just some BS to return something which COULD be it. untested 
    }

    if(state == GRINDING_GBW) { 
        stillToGrind = ((float)goalWeight/100 - currentWeight); 
        predictedTime = stillToGrind / (speedModifier*GBW_RPM_SET); 
    }

    return predictedTime;
}


void gbw_learn() { 

}