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

        // Update stored build ID
    } else firstBootFlag = false;
}

void pdata_read() {  //reads ALL stored settings in flash

    storedSetRPM = preferences.getShort("setRPM", default_setRPM);
    storedSetWeight = preferences.getULong("setWeight", default_setWeight);
    state = preferences.getShort("idleState", 1); 

    //Menu 1
    for(int i = 0; i < NUM_MENU1_ITEMS; i++) { 
        Menu1[i].value = preferences.getShort(Menu1[i].name, 0);
    }
    //Menu2
    for(int i = 0; i < NUM_MENU2_ITEMS; i++) { 
        Menu2[i].value = preferences.getShort(Menu2[i].name, 0);
    }
    //Menu 3
    for(int i = 0; i < NUM_MENU3_ITEMS; i++) { 
       Menu3[i].value = preferences.getShort(Menu3[i].name, 0);
    }
    preferences.getBytes("calibrateArray", calibrateArray, SIZEOFCALIBRATEARRAY); //raw data, works?

    setRPM = storedSetRPM;
    setWeight = storedSetWeight;
}


void pdata_write(uint8_t what) { // 0 = ALL SETTIGNS , 1 = Calibration array, 2 = specific setting (derived from state && menuXselected), 3 = setWeight or RPM (based on state), 4 = GbW speedModifier, 5 = lastState;

    if(what == 0) { //write ALL (default) settings *only used when first initalizing*
        //Menu 1
        for(int i = 0; i < NUM_MENU1_ITEMS; i++) { 
            preferences.putShort(Menu1[i].name, Menu1[i].value);
        }
        //Menu2
        for(int i = 0; i < NUM_MENU2_ITEMS; i++) { 
            preferences.putShort(Menu2[i].name, Menu2[i].value);
        }
        //Menu 3
        for(int i = 0; i < NUM_MENU3_ITEMS; i++) { 
            preferences.putShort(Menu3[i].name, Menu3[i].value);
        }

        preferences.putShort("setRPM", default_setRPM);
        preferences.putShort("setWeight", default_setWeight);
        preferences.putFloat("speedModifier", speedModifier);

        String currentBuildID = BUILD_ID;
        preferences.putString("build_id", currentBuildID);
        Serial.println("Stored ALL");
    }
    if(what == 1) { //write calibrateArray
        preferences.putBytes("calibrateArray", calibrateArray, SIZEOFCALIBRATEARRAY); //This puts raw data in 
        Serial.println("Stored 1");
    }

    if(what == 2) { 
        if(state == MENU1) preferences.putShort(Menu1[menu1Selected].name, Menu1[menu1Selected].value);
        if(state == MENU2) preferences.putShort(Menu2[menu2Selected].name, Menu2[menu2Selected].value);
        if(state == MENU3) preferences.putShort(Menu3[menu3Selected].name, Menu3[menu3Selected].value);
        Serial.println("Stored 2");
    }

    //update set RPM or weight
    if(what == 3) {
        if(state == IDLE) preferences.putShort("setRPM", setRPM);
        if(state == IDLE_GBW) preferences.putULong("setWeight", setWeight);
        Serial.println("Stored 3");
    }

    //update from gbw learn
    if(what == 4) { 
        if(state == GRINDING_GBW) preferences.putShort(Menu3[4].name, Menu3[4].value);
        if(state == GRINDING_GBW) preferences.putShort(Menu3[6].name, Menu3[6].value);  
        Serial.println("Stored 4");
    }

    //update idle state stored
    if(what == 5) { 
        if(state == IDLE) preferences.putShort("idleState", 1);
        if(state == IDLE_GBW) preferences.putShort("idleState", 2);
        Serial.println("Stored 5");
    }
}

