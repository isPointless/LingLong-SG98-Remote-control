#include "pdata.h"
#include "gbw.h"
#include "motorcontrol.h"
#include "main.h"

Preferences preferences; 

uint16_t settingsArray[20] = {0};
/* Settings: 
[0] = RPMset
[1] = weightSet
[2] = ViewMode
[3] = PurgeEnabled
[4] = ButtonPurge
[5] = purgeReverse
[6] = purgeFromZero 
[7] = purgeFramesLow
[8] = purgeFramesHigh
[9] = purgePrctLow
[10] = purgePrctHigh 
[11] = gbwRPMset
[12] = scaleAutoConnect
[13] = 
*/


void pdata_init() { 
    if (!preferences.begin("settings", false)) { // check if preferences is working correctly
        error = 103;
    } 
    preferences.end();
}

void pdata_read() { 
    preferences.begin("settings", true); 
    for(int i = 0; i < SIZEOFCALIBRATEARRAY/2; i++) { 
        preferences.getUShort("calibrateArray", calibrateArray[i]);
        if(i <= sizeof(settingsArray)) preferences.getUShort("settingsArray", settingsArray[i]);
    }
    preferences.end();
}


void pdata_write(uint8_t what, uint8_t specific) { // 0 = settings, 1 = calibrationarray, 2 = specific setting, only functional with 3, specific
    preferences.begin("settings", false); 

    if(what == 0) { //write settingsArray *why would we use this*
        for(int i = 0; i < sizeof(settingsArray)/sizeof(uint16_t); i++) { 
            preferences.putUShort("settingsArray", settingsArray[i]);
        }
    }
    if(what == 1) { //write calibrateArray
        for(int i = 0; i < SIZEOFCALIBRATEARRAY/sizeof(uint16_t); i++) { 
            preferences.putUShort("calibrateArray", calibrateArray[i]);
        } 
    }
    if(what == 3) { //write ONE setting
        preferences.putUShort("settingsArray", settingsArray[specific]);
    }

    preferences.end();
}