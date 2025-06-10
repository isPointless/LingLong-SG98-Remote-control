#include "definitions.h"
#ifdef RT_DRIVE
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

extern int16_t motor_setRPM;
extern int16_t motor_currentRPM;
extern uint16_t motor_currentTorque;
extern uint16_t motor_currentStatus;
extern int16_t calibrateArray[];


void motor_init(); //initialize / verify motor parameters 
// bool calibrate();
bool do_comm(); //returns true if new data has been received
bool motorOff();
bool Calibrate();

#endif

#endif