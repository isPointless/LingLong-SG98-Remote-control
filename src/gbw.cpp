#include "gbw.h"
#include <AcaiaArduinoBLE.h>
#include "main.h"
#include "motorcontrol.h"

AcaiaArduinoBLE scale(DEBUG);

uint16_t goalWeight = 180;
bool scaleConnected = false;

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

bool scales_connect() { 
    return true;
}

void gbwVitals() 
{   
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

    if(scale.isConnected() == false && state == GRINDING_GBW) { 
        state = IDLE_GBW;
        motorOff();
        error = 104;
        #ifdef DEBUG
        Serial.println("Scale disconnected while grinding by weight");
        #endif
    }

    #ifdef DEBUG
        Serial.print(currentWeight);
    #endif

}

