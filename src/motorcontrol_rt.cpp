#include "definitions.h"
#include "motorcontrol_rt.h"
#include "main.h"
#include "pdata.h"
#include "comm_rt.h"
#include "display.h"



int16_t motor_currentRPM = 0;
int16_t motor_currentTorque = 0;
uint16_t motor_currentStatus = 0;
int16_t motor_setRPM = 0;
uint16_t currentCal = 0;
RTC_DATA_ATTR uint8_t rtc_bootFlagMotor = 0;
int16_t commCounter = 0;

int16_t reg_201 = -1;
int16_t reg_503 = 0;
int16_t reg_514 = -1;
int16_t reg_515 = -1;
int16_t reg_525 = -1;


int16_t calibrateArray[SIZEOFCALIBRATEARRAY] = {0};

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
  // // do some fat blocking functions in init
  // if(rtc_bootFlagMotor == 1) { 
  //   return;
  // } else {

  // delay(50);
  // writeConfirm(0x04B8, 1); // reset faults
  // writeConfirm(0x006C, 0); // fault mode 2 behaviour: coast to stop, de energize
  // writeConfirm(0x006D, 0); // fault mode 1 behaviour: coast to stop, de energize
  // writeConfirm(0x0195, 200); // acc time constant 200ms (0-1000 rpm)
  // writeConfirm(0x0196, 200); // Deceleration time constant 200ms (1000>0 rpm)
  // writeConfirm(0x019C, absolute_max_rpm); // max speed 3000

  // rtc_bootFlagMotor = 1;
  // }

  // #ifdef DEBUG
  //  Serial.println("motor init complete");
  // #endif
}

bool do_comm() { // can be called continously and will update all values. Returns true when new data packet is available.
    /* COMM TYPES
    0 = send RPM
    1+ = READ STATUS, RPM, TORQUE (1300, 1301, 1302, 1303)
    */
  static unsigned long lastCommSend;
  static unsigned long lastCommReceived;
  static uint8_t lastCommType;
  static uint8_t nextCommType;
  static int16_t motor_lastSetRPM;

  if(lastCommReceived + DISCONNECTED_AFTER < millis()) { 
    error = 2;
    commStatus = DISCONNECTED;
  }

  //Polled continuously, self limiting 
  if(receive() == true) { 
    #ifdef DEBUG_COMM
      if(lastRequestType == 0 || lastRequestType > 200) {
        Serial.println("received!");
        Serial.print("reg 201: "), Serial.println(reg_201);
        Serial.print("reg 503: "), Serial.println(reg_503);
        Serial.print("reg 514: "), Serial.println(reg_514);
        Serial.print("reg 515: "), Serial.println(reg_515);
        Serial.print("reg 525: "), Serial.println(reg_525);
        Serial.print("next comm type: "), Serial.println(nextCommType); 
      }
    #endif

    commStatus = CONNECTED;
    lastCommReceived = millis(); 

    if(lastRequestType == 0) {  // a SEND
      //do nothing
    }

    if(lastRequestType == 104) { // A request (of 4 registers!)
        motor_currentStatus = lastRead[0];
        motor_currentRPM = lastRead[1];
        //skip 2
        motor_currentTorque = lastRead[3];
        newData = true;
        #ifdef DEBUG_COMM
          Serial.println("Newdata!");
        #endif
    }

    if(lastRequestType > 200 && lastRequestType < 205) { //A SEND of max 4 writes
      //do nothing
    }
    return true;
  } else newData = false; // We've made one complete round!

  // The only FAST & blocking service we're offering!
  if(motor_setRPM == 0 && motor_lastSetRPM != 0) { 
    if(reg_514 != 0) { 
      writeMultipleConfirm(514, 2, 0, 0); //SET SPEED LIMIT = 0
      reg_514 = 0;
    }
    motor_lastSetRPM = motor_setRPM;
    nextCommType = 0;
    return false;
  }

  if(motor_setRPM ==0) if(!reg_201 && !reg_503 && !reg_514 && !reg_515 && !reg_525) nextCommType = 1;

  //Determine COMM delay
  uint16_t delay;
  if (nextCommType == 0) delay = COMM_DELAY_SEND; 
  if(nextCommType == 1) { 
    if (state == IDLE || state == IDLE_GBW || state == MENU1 || state == MENU2 || state == MENU3) delay = COMM_DELAY_IDLE;
    else delay = COMMINTERVAL;
  }
  if(lastCommSend + delay > millis()) return false;

  // Continue sending::: 


  motor_lastSetRPM = motor_setRPM;

  if(commStatus == DISCONNECTED) { 
    if(writeMultipleConfirm(514, 2, 0, 0)) {
      reg_514 = 0;
      reg_515 = 0;
      nextCommType = 0;
      commStatus = CONNECTED;
      reg_525 = -1;
      reg_201 = -1;
      lastCommReceived = millis();
    } 
    lastCommSend = millis();
    return false;
  }

  if(motor_setRPM == 0) { 
      if(reg_514 != 0 || reg_515 != 0) { 
        writeMultipleRegisters(514, 2, 0, 0); //SET SPEED LIMIT FORWARD = 0
        lastCommSend = millis();
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_514 = 0;
          reg_515 = 0;
        }
        return false;
      }
      if(reg_525 != 0) { 
        writeSingleRegister(525, 0); // JOG DISABLE
        lastCommSend = millis();
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_525 = 0;
        }
        return false;
      }
      if(reg_201 != 0) { 
        writeSingleRegister(201, 0); //SERVO DISABLE
        Serial.println("Write 201");
        lastCommSend = millis(); 
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_201 = 0;
        }
        return false;
      }
      if(motor_currentRPM != 0 && lastActivity + 750 < millis()) { 
        reg_201 = reg_514 = reg_503 = reg_525 = -1; //Redo the whole shabang
      }
      // Check if we've done all commands.
      if(reg_201 == 0 && reg_514 == 0 && reg_525 == 0) nextCommType = 1;
  }

    //Forward
    if(motor_setRPM > 0) { 
      if(reg_201 != 1) { 
        writeSingleRegister(201, 1); // Servo ENABLED
        lastCommSend = millis(); 
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_201 = 1;
        }
        return false;
      }
      if(reg_503 != Menu1[SETMOTORTORQUE].value) { 
        writeSingleRegister(503, Menu1[SETMOTORTORQUE].value); // MOTOR TORQUE SET
        lastCommSend = millis(); 
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_503 = Menu1[SETMOTORTORQUE].value;
        }
        return false;
      }
      if(reg_514 != motor_setRPM || reg_515 != 0) { 
        writeMultipleRegisters(514, 2, motor_setRPM, 0); //SET SPEED LIMIT FORWARD
        lastCommSend = millis();
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_514 = motor_setRPM;
          reg_515 = 0;
        }
        return false;
      }
      if(reg_525 != 1) { 
        writeSingleRegister(525, 1); // JOG FORWARD
        lastCommSend = millis();
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_525 = 1;
        }
        return false;
      }
      //Check if we've done all commands
      if(reg_201 == 1 && reg_503 == Menu1[SETMOTORTORQUE].value && reg_514 == motor_setRPM && reg_515 == 0 && reg_525 == 1) nextCommType = 1; 
    } 

    //REVERSE
     if(motor_setRPM < 0) { 
      if(reg_201 != 1) { 
        writeSingleRegister(201, 1); // Servo ENABLED
        lastCommSend = millis(); 
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_201 = 1;
        }
        return false;
      }
      if(reg_503 != -Menu1[SETMOTORTORQUE].value) { 
        writeSingleRegister(503, -Menu1[SETMOTORTORQUE].value); // MOTOR TORQUE SET
        lastCommSend = millis(); 
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_503 = -Menu1[SETMOTORTORQUE].value;
        }
        return false;
      }
      if(reg_514 != 0 || reg_515 != -motor_setRPM) { 
        writeMultipleRegisters(514, 2, 0, -motor_setRPM); //SET SPEED LIMIT REVERSE, note motor_set is signed and sending UNSIGNED
        lastCommSend = millis();
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_514 = 0;
          reg_515 = -motor_setRPM;
        }
        return false;
      }
      if(reg_525 != 2) { 
        writeSingleRegister(525, 2); // JOG FORWARD
        lastCommSend = millis();
        lastCommType = 0;
        nextCommType = 0;
        if(commStatus == CONNECTED) {
          reg_525 = 2;
        }
        return false;
      }
      //Check if we've done all commands
      if(reg_201 == 1 && reg_503 == -Menu1[SETMOTORTORQUE].value && reg_514 == 0 && reg_515 == -motor_setRPM && reg_525 == 2) nextCommType = 1; 
  }

  if(nextCommType = 1) { 
    readRegister(1300, 4); //read register 1300, 1301, 1302, 1303
    lastCommSend = millis();
  }

  return false;
}

bool motorOff() {
  uint8_t score = 0;
    for(int i = 0; i < 10; i++) {
      delay(COMM_DELAY_SEND);
      if(writeMultipleConfirm(514, 2, 0, 0)) { //SET SPEED LIMIT FORWARD = 0
        reg_514 = 0;
        reg_515 = 0;
        score++;
        break; 
      } 
    }
    for(int i = 0; i < 10; i++) {
      delay(COMM_DELAY_SEND);
      if(writeConfirm(525, 0)) { // JOG DISABLE
        reg_525 = 0;
        score++;
        break;
      }
    }
    for(int i = 0; i < 10; i++) {
      delay(COMM_DELAY_SEND);
      if (writeMultipleConfirm(201, 1, 0)) { //SERVO DISABLE
        reg_201 = 0;
        score++;
        break;
      }
    }
    if(score >=3) return true;
    else return false;
    return false;
}

//This function needs to be called continuously untill it returns true; which indicates a completed calibration
bool Calibrate() { 
    static unsigned long lastCall;
    static unsigned long checkTime;
    static uint32_t sum = 0;
    static uint16_t sumCount = 0;

    if(lastCall + 3000 < millis()) { //reset after last call 2.5s ago. 
        currentCal = 0; 
        sum = 0;
        sumCount = 0;
        checkTime = millis();
    }

    lastCall = millis(); 

    if(currentCal == 0 && Menu1[SETMIN].value == 0) { 
        calibrateArray[0] = 0;
        currentCal = 1;
        checkTime = millis();
    }

    if(commStatus != CONNECTED) { //Can't go here if comm is bad.
        error = 101; 
        state = MENU1;
        return false;
    }

    //All during calibration actions
    if(checkTime + 250 < millis() && newData == true) { // Collect current frames after 250 ms of stabilizations
        sum = sum + motor_currentTorque;
        sumCount++;
    }

    //All progression to next RPM actions
    if(currentCal < (SIZEOFCALIBRATEARRAY/sizeof(uint16_t)) && checkTime + CAL_TIME < millis() && sumCount != 0) { 
        calibrateArray[currentCal] = uint16_t(sum/sumCount);
        #ifdef DEBUG_CALIB
            Serial.print("Progressing!: "), Serial.println(currentCal);
            Serial.print("sumcount: "), Serial.println(sumCount);
            Serial.print("sum: "), Serial.println(sum);
            Serial.print("Av torq: "), Serial.println(sum/sumCount);
        #endif
        currentCal++;
        motor_setRPM = (currentCal*rpm_scalar + absolute_min_rpm) > absolute_max_rpm ? absolute_max_rpm : (currentCal*rpm_scalar + absolute_min_rpm);
        checkTime = millis(); 
        sum = 0;
        sumCount = 0;
        disp_updateRequired = true;
    }

    //Ending actions
    if(checkTime + CAL_TIME < millis() && currentCal >= (SIZEOFCALIBRATEARRAY/sizeof(uint16_t))) { //we have completed the last Calibration
        if(sumCount != 0) calibrateArray[currentCal] = uint16_t(sum/sumCount);
        motor_setRPM = 0;
        pdata_write(1);
        #ifdef DEBUG_CALIB
        Serial.println("Writing calib array!"); 
        for(int i = 0; i < SIZEOFCALIBRATEARRAY/2; i++) { 
          Serial.print("Array"), Serial.print(i), Serial.print(" value: "), Serial.println(calibrateArray[i]);
        }
        #endif
        state = IDLE;
        enterMenu = false; 
        menu1Selected = EXITMENU;
        return true;
    }

    return false;
}
