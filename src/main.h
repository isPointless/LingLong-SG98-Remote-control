#include "definitions.h"

#ifndef MAIN_H
#define MAIN_H

void vitals_init();
void doVitals();
void sleep_init();
void sleep_release();
void sleepTimeCheck();
void checkPurge();
void do_purge();

enum States{SLEEPING, IDLE, IDLE_GBW, GRINDING, PURGING, GRINDING_GBW, MENU1, MENU2, MENU3, CALIBRATING};
enum Menu1Items{EXITMENU, DISP_BRIGHTNESS, PURGESETTINGS, GBWSETTINGS, CALIBRATE, SETMAX, SETMIN, SETSLEEP, SETMOTORTORQUE, LED_BRIGHTNESS, INVERT_SCROLL, NUM_MENU1_ITEMS};
enum Menu2Items{RETURN_FROM_PURGE, SETBUTTONPURGE, SETPURGEFRAMESLOW, SETPURGEFRAMESHIGH, SETPURGEPRCTLOW, SETPURGEPRCTHIGH, SETPURGETIME, SETPURGEDELAY, SETPURGEREVERSE, AUTO_PURGE_ENABLED, SETPURGESTABILTIME, SETPURGERPM, SETPURGEOFFTIME, NUM_MENU2_ITEMS};
enum Menu3Items{RETURN_FROM_GBW, GBW_RPM_SET, GBW_SLOW_PHASE, GBW_SLOW_RPM, GBW_SPEEDMOD, GBW_BUTTON_DELAY, GBW_OFFSET, SAVE_SCALE, NUM_MENU3_ITEMS};

struct menuEntry { 
    const char* name;
    int16_t value;
    int16_t minValue;
    int16_t maxValue;
    int16_t scalar;
    const char* prefUID;
};

extern menuEntry Menu1[NUM_MENU1_ITEMS];
extern menuEntry Menu2[NUM_MENU2_ITEMS];
extern menuEntry Menu3[NUM_MENU3_ITEMS];

extern bool firstBootFlag;

extern States state;

extern bool modeGbW;
extern int8_t menu1Selected;
extern int8_t menu2Selected;
extern int8_t menu3Selected;
extern bool enterMenu;

extern int16_t setRPM;
extern uint32_t setWeight; //in mg (18g = 18000)

extern unsigned long lastActivity;

extern bool ready_purge;

/* errors: 0 = no error 
FATAL: 
1 = cant connect
2 = connection was present but failed 
3 = Error frame received / drive fault
4 = Comm discrepancy over 100
5 = Failed to create mutex
6 = motor stalled

NON FATAL: 
100 = motor set RPM not equal to read from controller
101 = calibration couldnt start due to... 
102 = calibration cancelled
103 = couldnt load preferences
104 = scale disconnected during grinding
105 = GbW not producing any grounds after 3s
106 = GbW couldn't start -> no scale connected


*/
extern volatile uint8_t error;
extern volatile bool newData;
extern uint16_t sleepTime;

#endif