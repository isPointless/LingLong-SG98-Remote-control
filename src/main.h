#include "definitions.h"

#ifndef MAIN_H
#define MAIN_H

void vitals_init();
void doVitals();
void sleep_init();
void sleep_release();
void updateSleepTimer();
void checkPurge();

enum States{SLEEPING, IDLE, IDLE_GBW, GRINDING, PURGING, GRINDING_GBW, MENU1, MENU2, CALIBRATING};
enum Menu1Items{EXIT, VIEWMODE, PURGESETTINGS, GBWSETTINGS, CALIBRATE, SETMAX, SETMIN, SETSLEEP};
enum Menu2Items{SETBUTTONPURGE, SETPURGEFRAMESLOW, SETPURGEFRAMESHIGH, SETPURGEPRCTLOW, SETPURGEPRCTHIGH, SETPURGETIME, SETPURGEDELAY, SETPURGEREVERSE};
enum SYSTEM_STATUS{DISCONNECTED, ERROR, CONNECTED, READY};

extern uint8_t commStatus;
extern uint8_t state;
extern bool modeGbW;
extern int8_t menu1Selected;
extern int8_t menu2Selected;
extern bool enterMenu;
extern int16_t setRPM;
extern uint16_t setWeight; //in g*10 (18.5g = 185)
extern bool purge_enabled; 

extern unsigned long lastActivity;

/* errors: 0 = no error 
FATAL: 
1 = cant connect
2 = connection was present but failed 
3 = Error frame received
NON FATAL: 
100 = motor set RPM not equal to read from controller
101 = calibration couldnt start due to... 
102 = calibration cancelled
103 = couldnt load preferences
104 = scale disconnected during grinding


*/
extern uint8_t error;
extern bool newData;
extern uint16_t sleepTime;

#endif