#include "definitions.h"

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

extern int16_t motor_setRPM;
extern int16_t motor_currentRPM;
extern int16_t motor_currentTorque;
extern uint16_t motor_currentStatus;
extern int16_t calibrateArray[];
extern uint16_t currentCal;
extern int16_t commCounter;


void motor_init(); //initialize / verify motor parameters 
// bool calibrate();
bool do_comm(); //returns true if new data has been received
bool motorOff();
bool Calibrate();

#endif