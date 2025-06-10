#include "definitions.h"

#ifndef gbw_h
#define gbw_h

#define DEBUG false

extern int counter;
extern float currentWeight;

void scales_init();
void gbwVitals();
bool scale_connected();
uint16_t gbw_predict();
void gbw_learn();

#endif