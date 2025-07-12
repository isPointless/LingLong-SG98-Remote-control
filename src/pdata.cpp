#include <nvs_flash.h>
#include "pdata.h"
#include "gbw.h"
#include "main.h"
#include "gbw.h"
#include "io.h"

#ifdef JMC_DRIVE
#include "motorcontrol_jmc.h"
#endif

#ifdef RT_DRIVE
#include "motorcontrol_rt.h"
#endif

Preferences preferences; 


void pdata_init() { 
    preferences.begin("settings", false);

    // Get stored firmware build ID
    String storedBuildID = preferences.getString("build_id", "");
    String currentBuildID = BUILD_ID;

    if (storedBuildID != currentBuildID) {
        // First boot OR firmware changed
        firstBootFlag = true;
        preferences.end();
        delay(200);
        nvs_flash_erase();   // Erase the entire flash partition used for NVS
        nvs_flash_init();    // Re-initialize NVS
        Serial.println("First boot detected! Saving ALL");
        delay(200);
        preferences.begin("settings", false);
        // Update stored build ID
    } else firstBootFlag = false;
}

void pdata_read() {  //reads ALL stored settings in flash

    storedSetRPM = preferences.getShort("setRPM", default_setRPM);
    storedSetWeight = preferences.getULong("setWeight", default_setWeight);
    state = preferences.getShort("idleState", 1); 
    if(state == SLEEPING) state = IDLE;

    //Menu 1
    for(int i = 0; i < NUM_MENU1_ITEMS; i++) { 
        Menu1[i].value = preferences.getShort(Menu1[i].prefUID, Menu1[i].value);
    }
    //Menu2
    for(int i = 0; i < NUM_MENU2_ITEMS; i++) { 
        Menu2[i].value = preferences.getShort(Menu2[i].prefUID, Menu2[i].value);
    }
    //Menu 3
    for(int i = 0; i < NUM_MENU3_ITEMS; i++) { 
       Menu3[i].value = preferences.getShort(Menu3[i].prefUID, Menu3[i].value);
    }
    preferences.getBytes("calibrateArray", calibrateArray, SIZEOFCALIBRATEARRAY); //raw data, works?

    #ifdef DEBUG_CALIB
    for(int i = 0; i < (SIZEOFCALIBRATEARRAY/2); i++) { 
        Serial.print("Array"), Serial.print(i), Serial.print(" value: "), Serial.println(calibrateArray[i]);
    }
    #endif
    // //Retrieve stored scale info. 
    String scale_name = "", scale_mac = "";
    scale_connect_name = preferences.getString("SCALE_NAME", "");
    scale_connect_mac = preferences.getString("SCALE_MAC", "");

    //setrpm, weight
    setRPM = storedSetRPM;
    setWeight = storedSetWeight;
}


void pdata_write(uint8_t what) { // 0 = ALL SETTIGNS , 1 = Calibration array, 2 = specific setting (derived from state && menuXselected), 3 = setWeight or RPM (based on state), 4 = GbW speedModifier, 5 = lastState;

    if(what == 0) { //write ALL (default) settings *only used when first initalizing*
        //Menu 1
        for(int i = 0; i < NUM_MENU1_ITEMS; i++) { 
            preferences.putShort(Menu1[i].prefUID, Menu1[i].value);
        }
        //Menu2
        for(int i = 0; i < NUM_MENU2_ITEMS; i++) { 
            preferences.putShort(Menu2[i].prefUID, Menu2[i].value);
        }
        //Menu 3
        for(int i = 0; i < NUM_MENU3_ITEMS; i++) { 
            preferences.putShort(Menu3[i].prefUID, Menu3[i].value);
        }

        preferences.putShort("setRPM", default_setRPM);
        preferences.putShort("setWeight", default_setWeight);
        preferences.putFloat("speedModifier", speedModifier);

        preferences.putString("SCALE_NAME", "");
        preferences.putString("SCALE_MAC", "");

        String currentBuildID = BUILD_ID;
        preferences.putString("build_id", currentBuildID);
        Serial.println("Stored ALL");

        //set array to 11 
        for(int i = 0; i < SIZEOFCALIBRATEARRAY; i++) { 
            calibrateArray[i] = 11;
        }
        preferences.putBytes("calibrateArray", calibrateArray, SIZEOFCALIBRATEARRAY); //This puts raw data in
    }
    if(what == 1) { //write calibrateArray
        preferences.remove("calibrateArray");
        preferences.putBytes("calibrateArray", calibrateArray, SIZEOFCALIBRATEARRAY); //This puts raw data in 
        #ifdef DEBUG_CALIB
        Serial.println("Stored 1");
        #endif
    }

    if(what == 2) { 
        if(state == MENU1) preferences.putShort(Menu1[menu1Selected].prefUID, Menu1[menu1Selected].value);
        if(state == MENU2) preferences.putShort(Menu2[menu2Selected].prefUID, Menu2[menu2Selected].value);
        if(state == MENU3) preferences.putShort(Menu3[menu3Selected].prefUID, Menu3[menu3Selected].value);
        //Serial.println("Stored 2");
    }

    //update set RPM or weight
    if(what == 3) {
        if(state == IDLE) preferences.putShort("setRPM", setRPM);
        if(state == IDLE_GBW) preferences.putULong("setWeight", setWeight);
        //Serial.println("Stored 3");
    }

    //update from gbw learn
    if(what == 4) { 
        if(state == GRINDING_GBW) preferences.putShort(Menu3[4].prefUID, Menu3[4].value);
        if(state == GRINDING_GBW) preferences.putShort(Menu3[6].prefUID, Menu3[6].value);  
       // Serial.println("Stored 4");
    }

    //update idle state stored
    if(what == 5) { 
        if(state == IDLE) preferences.putShort("idleState", 1);
        if(state == IDLE_GBW) preferences.putShort("idleState", 2);
       // Serial.println("Stored 5");
    }

    // Remove scale info
    if(what == 6) { 
        if(state != MENU3) return;
            preferences.putString("SCALE_NAME", "");
            preferences.putString("SCALE_MAC", "");
    }

    // save scale info
    if(what == 7) { 
        String macCopy;
        String nameCopy;
        if (xSemaphoreTake(scaleMutex, portMAX_DELAY)) {
            macCopy = scale_connect_mac;
            nameCopy = scale_connect_name;
            xSemaphoreGive(scaleMutex);
        } 
        preferences.putString("SCALE_NAME", nameCopy);
        preferences.putString("SCALE_MAC", macCopy);
    }
}

