// pin definitions and settings
#include <Arduino.h>
#pragma once

//DEBUG FLAGS
#define DEBUG_SCALE false
// #define DEBUG_COMM
// #define DEBUG_CALIB
// #define DEBUG_GBW

//#define BUILD_ID __DATE__ __TIME__
#define BUILD_ID "0002"

// USER SETTINGS
#define absolute_min_rpm 0
#define absolute_max_rpm 2500
#define rpm_scalar 25
#define SIZEOFCALIBRATEARRAY (2*(absolute_max_rpm - absolute_min_rpm)/rpm_scalar) // 2 * (ABSOLUTE_MIN_RPM - ABSOLUTE_MAX_RPM) / RPM_SCALAR
#define LONGPRESS 500 //after Xms a long press is registered
#define RT_DRIVE
// #define JMC_DRIVE

// COMM PINS
#define RS485_RXD 18 
#define RS485_TXD 17
#define RS485RE 15
#define RS485DE 16

// IO PINS
#define BTN 4
#define start_button_led 5
#define ENC_BTN 42
#define ENC_A 41
#define ENC_B 40

// SPI display PINS
#define DISP_SCL 10
#define DISP_SDA 11
#define DISP_RST 12
#define DISP_DC 13
#define DISP_CS 14
#define DISP_BL 21

#define SLEEP_DISP_BL GPIO_NUM_14

//EX_SPI (unused but accessible in the board)
#define EXSPI_MOSI 35
#define EXSPI_MISO 37
#define EXSPI_CLK 36
#define EXPSI_CS 39

//EX I2C (unused but accessible)
#define EXSDA 8
#define EXSCL 9

// COMM SETTINGS
#define COMMINTERVAL 100 // comm interval during normal operation
#define COMM_DELAY_RECEIVE 50 //expect a respond 10ms after request send
#define COMM_DELAY_IDLE 1000
#define COMM_DELAY_SEND 50 //send every 50ms max 
#define DISCONNECTED_AFTER 10000 //after Xms of not receiving comms, disconnected error shows
#define SEND_RPM_EVERY 5 //send RPM every X frames

// These must be the same as below!
#define WAKE_BTN GPIO_NUM_4
#define WAKE_ENC_BTN GPIO_NUM_42
#define WAKE_ENC_A GPIO_NUM_41
#define WAKE_ENC_B GPIO_NUM_40
#define SLEEP_RE GPIO_NUM_15
#define SLEEP_DE GPIO_NUM_16


//Encoder ticks per physical tick
#define ENC_TOL 1

//led behaviour
#define fadeTime 1750
#define flashTime 250
#define fastFlashTime 150
#define fastestFlashTime 75

// defaults
#define default_sleepTime 120 //in s
#define CAL_TIME 2500 //calibration time minus 500ms of settling in
#define default_setRPM 1000
#define default_brightness 200
#define default_maxRPM 2500
#define default_minRPM 100
#define default_motor_torque 1040

#define default_setWeight 18000
#define default_max_weight 99900
#define default_min_weight 6000
#define default_GBWRPM 600
#define default_slow_time 400
#define default_slow_rpm 200
#define default_speedModifier 4500
#define default_start_delay 200
#define default_time_offset 300


#define default_autoPurgeEnabled 1
#define default_buttonPurge 1
#define default_autoPurgeEnabled 1
#define default_purgeFramesLow 5
#define default_purgeFramesHigh 3
#define default_purgePrctLow 120
#define default_purgePrctHigh 120
#define default_purgeDuration 3000
#define default_purgeDelay 2000
#define default_purgeForwardRPM 2000
#define default_reverseRotation 0
#define default_purgeStabilTime 500