#include "definitions.h"
#include <BLEScale.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef gbw_h
#define gbw_h

extern BLEScale scale;

//#define DEBUG false

//extern int counter;
extern float speedModifier;

extern volatile bool grindingComplete;
extern volatile bool gbw_started;
extern volatile bool scaleConnected;
extern volatile int32_t currentWeight;
extern volatile bool saveScaleEnabled;

extern String scale_mac;
extern String scale_name;
extern String scale_connect_mac;
extern String scale_connect_name;

extern SemaphoreHandle_t scaleMutex;

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