#include "definitions.h"
#ifdef JMC_DRIVE
#include "motorcontrol_jmc.h"
#include "main.h"
#include "pdata.h"



#ifdef RT_DRIVE 
#include "comm_rt.h"
#endif 
#ifdef JMC_DRIVE
#include "comm_jmc.h"
#endif


int16_t motor_currentRPM = 0;
uint16_t motor_currentTorque = 0;
uint16_t motor_currentStatus = 0;
int16_t motor_setRPM = 0;
RTC_DATA_ATTR uint8_t rtc_bootFlagMotor = 0;

int16_t calibrateArray[((absolute_max_rpm-absolute_min_rpm)/rpm_scalar)] = {0};

/* Important motor parameters for RT, note this is decimal, and should be communicated in HEX (below is bin)
0022 -> motor rated torque

0100 -> control mode (1 = speed mode)
0108 & 0109 -> fault mode behaviour (0 = coast & de energize)
0400 -> command source A input type
0402 -> command source selection
0403 -> "digital speed given" -> control RPM with this value (signed value)
0405 -> accel time constant (time for 0 -> 1000 rpm) in ms
0406 -> decel time constant (time for 1000 -> 0 rpm)
0408 -> motor rotational speed -> is this monitoring? 
0412, 0413, 0414 -> max speed, max forward speed, max reverse speed 
1300 -> running status
1301 -> current RPM
1303 -> motor torque
1353 -> internal fault code -> no clue what is what ??

*/

void motor_init() {

  while(readReturn(0x0514, 1) == (-1)) delay(100); 

  if(rtc_bootFlagMotor == 1) { 
    return;
  } else {

  delay(50);
  writeConfirm(0x04B8, 1); // reset faults
  writeConfirm(0x0193, 0); //0 speed given
  writeConfirm(0x0064, 1); // set speed control mode
  writeConfirm(0x006C, 0); // fault mode 2 behaviour: coast to stop, de energize
  writeConfirm(0x006D, 0); // fault mode 1 behaviour: coast to stop, de energize
  writeConfirm(0x0190, 0); // Command source A = digital speed given
  writeConfirm(0x0192, 0); // command source A = actual
  writeConfirm(0x0195, 200);// acc time constant 200ms (0-1000 rpm)
  writeConfirm(0x0196, 200); // Deceleration time constant 200ms (1000>0 rpm)
  writeConfirm(0x019C, 3000); // max speed 3000
  //writeSingleRegister(0x0817, 0); // disable virtual IN?

  rtc_bootFlagMotor = 1;
  }

  #ifdef DEBUG
   Serial.println("motor init complete");
  #endif
}

bool do_comm() { // can be called continously and will update all values
    /* COMM TYPES
    0 = send RPM
    1+ = READ STATUS, RPM, TORQUE (1300, 1301, 1302, 1303)
    */
  static unsigned long lastCommSend;
  static unsigned long lastCommReceived;
  static uint8_t lastCommType;
  static uint8_t nextCommType;
  static int16_t motor_lastSetRPM;

  if(receive() == true) { 
    if(lastRequestType == 0) {  // we only SEND motor RPM as command during cyclic comm
        //do nothing -> this just confirms the set value.
    }

    if(lastRequestType == 104) {   //Cyclic status reading
        motor_currentStatus = lastRead[0];
        motor_currentRPM = lastRead[1];
         if(lastRead[2] != motor_setRPM) { 
            error = 100;
            nextCommType = 0;
        }
        motor_currentTorque = lastRead[3];
        newData = true;
    }
    lastCommReceived = millis();
    return true;
  }

  //prioritize a set command over cyclic read
  
  if(lastCommSend + (motor_lastSetRPM != motor_setRPM? COMM_DELAY : COMMINTERVAL) > millis()) return false;

  //Send new RPM if 
  if(nextCommType == 0 || motor_lastSetRPM != motor_setRPM) { 
    writeSingleRegister(0x0193, setRPM); //register 403
    motor_lastSetRPM = motor_setRPM;
    lastCommType = 0;
    lastCommSend = millis();
  }

  if(nextCommType > 0) { 
    readRegister(0x0514, 4); //read register 1300, 1301, 1302, 1303
    lastCommType = 1;
    lastCommSend = millis();
  }

  if(lastCommReceived + DISCONNECTED_AFTER < millis()) { 
    error = 2;
    commStatus = DISCONNECTED;
  }

  nextCommType++;
  if(nextCommType > SEND_RPM_EVERY) nextCommType = 0;

  return true;
}

bool motorOff() { //High priority turn off, pauses everything else for this comm
    motor_setRPM = 0;
    if(writeConfirm(0x0193, motor_setRPM) == true) { 
        return true;
    } else return motorOff(); //recursive... will stop at nothing
}

//This function needs to be called continuously untill it returns true; which indicates a completed calibration
bool Calibrate() { 
    static unsigned long checkTime;
    static uint16_t currentCal = 0; //calibration points 0 to x
    static uint32_t sum = 0;
    static uint8_t sumCount = 0;

    if(checkTime + (4*CAL_TIME) < millis()) { //reset after last call 10s ago. 
        currentCal = 0; 
        sum = 0;
        sumCount = 0;
    }

    if(commStatus != READY || currentCal == 0 && motor_currentRPM != 0) { //Can't go here if motor RPM != 0 or comm not present
        error = 101; 
        return false;
    }

    //All during calibration actions
    if(checkTime + 500 < millis() && newData == true) { //First new data frame after 500ms on this new RPM
        sum += motor_currentTorque;
        sumCount++;
    }

    //All progression to next RPM actions
    if(currentCal < sizeof(calibrateArray)/sizeof(uint16_t) && checkTime + CAL_TIME < millis()) { 
        calibrateArray[currentCal] = uint16_t(sum/sumCount);
        currentCal++;
        motor_setRPM = (currentCal*rpm_scalar + absolute_min_rpm) > absolute_max_rpm ? absolute_max_rpm : (currentCal*rpm_scalar + absolute_min_rpm);
        checkTime = millis(); 
    }

    //Ending actions
    if(checkTime + CAL_TIME < millis() && currentCal == sizeof(calibrateArray)/sizeof(uint16_t)) { //we have completed the last Calibration
        calibrateArray[currentCal] = uint16_t(sum/sumCount);
        motor_setRPM = 0;
        pdata_write(1,0);
        motorOff();
        return true;
    }
    return false;
}


#endif