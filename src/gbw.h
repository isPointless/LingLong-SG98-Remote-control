#include "definitions.h"

#ifndef gbw_h
#define gbw_h

//#define DEBUG false

//extern int counter;
extern float speedModifier;

extern volatile bool grindingComplete;
extern volatile bool gbw_started;
extern volatile bool scaleConnected;
extern volatile int32_t currentWeight;

struct GBW_WEIGHT { 
    uint32_t time;
    int32_t weight; //in MG
};

extern GBW_WEIGHT _shot[];

enum SCALE_STATUS {SCALE_DISCONNECTED, SCALE_CONNECTED, SCALE_CONNECTING, SCALE_LOST, INVALID_SCALE_STATUS};

extern volatile SCALE_STATUS scaleStatus; 

void scales_init();
void gbwVitals();
int16_t fakeScale();
bool scale_connected();
int16_t gbw_predict();
void gbw_learn();
void do_gbw();

#endif