#include "definitions.h"
#ifdef JMC_DRIVE

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

extern int16_t motor_setRPM;
extern int16_t motor_currentRPM;
extern int16_t motor_currentTorque;
extern int16_t calibrateArray[];
extern uint16_t currentCal;
extern int16_t commCounter;

enum COMM_STATUS{COMM_DISCONNECTED, COMM_CONNECTED, COMM_INVALID_STATUS};
extern COMM_STATUS commStatus;

enum SYSTEM_STATUS{MOTOR_NOT_CONNECTED, MOTOR_NOT_READY, MOTOR_READY, MOTOR_ENABLED, MOTOR_FAULT, MOTOR_INVALID};
extern SYSTEM_STATUS currentStatus;

void motor_init(); //initialize / verify motor parameters 
bool do_comm(); //returns true if new data has been received
bool motorOff();
bool Calibrate();
void motorSleep();
void motorReset();

#endif
#endif