// pin definitions and settings
#include <Arduino.h>
#pragma once

// USER SETTINGS
#define absolute_min_rpm 100
#define absolute_max_rpm 3000
#define rpm_scalar 25
#define SIZEOFCALIBRATEARRAY 232 
#define LONGPRESS 500

// COMM SETTINGS
#define BAUD_RATE 115200 //default for RT
#define PARITY SERIAL_8N1 //default for RT
#define RXD 18 
#define TXD 17
#define COMMINTERVAL 100
#define COMM_DELAY 30
#define COMM_DELAY_SEND
#define DISCONNECTED_AFTER 1000 //after 1000ms of not receiving comms, disconnected error shows
#define SEND_RPM_EVERY 5 //send RPM every 5 frames

// These must be the same as below!
#define WAKE_BTN GPIO_NUM_4
#define WAKE_ENC_BTN GPIO_NUM_5
#define WAKE_ENC_A GPIO_NUM_6
#define WAKE_ENC_B GPIO_NUM_7
#define SLEEP_RE GPIO_NUM_1
#define SLEEP_DE GPIO_NUM_2

#define BTN 4
#define ENC_BTN 5
#define ENC_A 6
#define ENC_B 7
#define RS485RE 1
#define RS485DE 2
#define start_button_led 15

#define ENC_TOL 4


//led behaviour
#define fadeTime 1750
#define flashTime 250
#define fastFlashTime 150
#define fastestFlashTime 75

// defaults
#define default_sleeptime 600 //in s
#define CAL_TIME 2500 //calibration time minus 500ms of settling in
#define default_setRPM 1000
#define default_weight 180