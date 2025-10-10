#include "definitions.h"

#ifdef JMC_DRIVE
#include "motorcontrol_jmc.h"
#include "main.h"
#include "pdata.h"
#include "comm.h"
#include "display.h"


int16_t motor_currentRPM = 0;
int16_t motor_currentTorque = 0;
uint16_t motor_currentStatus = 0;
int16_t motor_setRPM = 0;
uint16_t currentCal = 0;
RTC_DATA_ATTR uint8_t rtc_bootFlagMotor = 0;
int16_t commCounter = 0;

unsigned long lastCommSend;

COMM_STATUS commStatus = COMM_DISCONNECTED;
SYSTEM_STATUS currentStatus = MOTOR_NOT_CONNECTED;

int16_t calibrateArray[SIZEOFCALIBRATEARRAY] = {0};

int16_t reg_6040 = -1; //Control word

int16_t reg_0101 = -1; //mode torque = 2;
int16_t reg_0121 = -1; //stop torque %
int16_t reg_0102 = -1; //Rigidity source selection
int16_t reg_0103 = -1; //rigidity amount (0-31)

int16_t reg_0500 = -1; // torq command source = 1
int16_t reg_0501 = -1; // speed limit source = 1
int16_t reg_0502 = -1; // speed limit setpoint = RPM (uint)
int16_t reg_0503 = -1; // torque SET POINT (-300 to 300)
int16_t reg_0510 = -1; // internal fw torque limit (int 0-300)
int16_t reg_0511 = -1; // internal rv torque limit (int -300 - 0)

enum DriveStatus{
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED,
    FAULT,
    FAULT_REACTION_ACTIVE,
    QUICK_STOP_ACTIVE,
    UNKNOWN_STATUS
};

DriveStatus decodeStatusWord(uint16_t statusWord) {
    // Extract relevant bits (0-6) from the status word
    uint8_t stateBits = statusWord & 0x006F; // bitmask 0b0000000001101111 (0, 1, 2, 3, 5, and 6)

    switch (stateBits) {
        case 0x0000: // 0b0000000
            return SWITCH_ON_DISABLED;
        case 0x0021: // 0b0100001
            return READY_TO_SWITCH_ON;
        case 0x0023: // 0b0100011
            return SWITCHED_ON;
        case 0x0027: // 0b0100111
            return OPERATION_ENABLED;
        case 0x0008: // 0b0001000
            return FAULT;
        case 0x000F: // 0b0001111
            return FAULT_REACTION_ACTIVE;
        case 0x0007: // 0b0000111
            return QUICK_STOP_ACTIVE;
        default:
            return UNKNOWN_STATUS;
    }
}

SYSTEM_STATUS changeToStatus(DriveStatus decodedStatusWord) { 
    switch(decodedStatusWord) { 
      case SWITCH_ON_DISABLED: return MOTOR_NOT_READY;
      case READY_TO_SWITCH_ON: return MOTOR_NOT_READY;
      case SWITCHED_ON: return MOTOR_READY;
      case OPERATION_ENABLED: return MOTOR_ENABLED;
      case FAULT: return MOTOR_FAULT;
      case FAULT_REACTION_ACTIVE: return MOTOR_FAULT;
      case QUICK_STOP_ACTIVE: return MOTOR_FAULT;
      default: return MOTOR_INVALID;
    }
}

void motor_init() {
  motor_setRPM = 0;
  reg_6040 = -1; //Control word

  reg_0500 = -1; // torq command source = 1
  reg_0501 = -1; // speed limit source = 1
  reg_0502 = -1; // speed limit setpoint = RPM (uint)
  reg_0503 = -1; // torque SET POINT (-300 to 300)
  reg_0510 = -1; // internal fw torque limit (int 0-300)
  reg_0511 = -1; // internal rv torque limit (int -300 - 0)
  
  commStatus = COMM_DISCONNECTED;
  currentStatus = MOTOR_NOT_CONNECTED;
}

bool do_comm() { // can be called continously and will update all values. Returns true when new data packet is available.
    /* COMM TYPES
    0 = send RPM
    1 = read status word 0x6041
    2+ = READ STATUS, RPM, TORQUE (1300, 1301, 1302, 1303)
    */
  static unsigned long lastCommReceived;
  // static uint8_t lastCommType;
  static uint8_t nextCommType;
  static int16_t motor_lastSetRPM = -1;

  //Disconnect flow
  if(lastCommReceived + 2*COMM_DELAY_IDLE < millis() && lastCommReceived > 0) { 
    #ifndef DEBUG
      error = 2;
    #endif

    #ifdef DEBUG_MOTORCONTROL
      Serial.print("Have not received a message for: "), Serial.print(millis()-lastCommReceived), Serial.println("ms -> DISCONNECTED");
    #endif
    commStatus = COMM_DISCONNECTED;
    currentStatus = MOTOR_NOT_CONNECTED;
    disp_updateRequired = true;

    reg_6040 = -1; //Control word
    reg_0502 = -1; //speed
    reg_0503 = -1; //torque
  }

//Cannot establish connection 
  if(lastCommReceived == 0 && commStatus == COMM_DISCONNECTED && millis() > 10000) { 
    #ifndef DEBUG
      error = 1;
    #endif
    currentStatus = MOTOR_NOT_CONNECTED;
    disp_updateRequired = true;
  }

  //Unstable connection flow
  if(commStatus != COMM_DISCONNECTED && commCounter > 25 ) { 
    error = 4;
    motor_setRPM = 0;
    state = IDLE;
    disp_updateRequired = true;
  }

  // Motor stall flow
  int16_t torquePercent;
  if((Menu1[SETMOTORTORQUE].value) > 0) torquePercent = 10 * (motor_currentTorque / Menu1[SETMOTORTORQUE].value); //A value 
  else torquePercent = 0;
  if(motor_setRPM != 0 && torquePercent > 90 && abs(motor_currentRPM) < rpm_scalar) { 
    motor_setRPM = 0;
    state = IDLE; 
    error = 6;
  }

  // First communication when disconnected
  if(commStatus == COMM_DISCONNECTED) { 

    if(lastCommSend + COMM_DELAY_SEND > millis()) return false;

    if(writeConfirm(0x6040, 1)) { //This is blocking

      #ifdef DEBUG_MOTORCONTROL
        Serial.println("We're connected!.. ");
      #endif
      readReturn(0x1010, 2); //read magic byte setting MSB 
      // Check if magic byte is already set
      if(lastRead[0] != 0x6576 && lastRead[1] != 0x6173) { 
        if(!writeMultipleConfirm(0x1010, 2, 0x6576, 0x6173)) return false;  // magic byte "Save" to 0x1010 
        if(readReturn(0x2101, 1) != 2) error = 8;   //Confirm we've already written data or not.. to recognize a first boot event.

        int score = 0; 
        if(writeConfirm(0x2101, 2)) { reg_0101 = 2; score++; } //Set mode to torque control

        // Motor rigidity settings.. automatic mode with 0 gain and 2000 filter.
        if(writeConfirm(0x2102, 1)) { reg_0102 = 1; score++; } // MODE 1 - semi auto
        if(writeConfirm(0x2103, Menu1[SETMOTORRAMP].value)) { reg_0103 = Menu1[SETMOTORRAMP].value; score++; } //P103 set, this is set in the MENU

        if(writeConfirm(0x2121, 30)) { reg_0121 = 30;  score ++; } //This is the quickstop ramp down torque (currently not used i believe the way we use it)

        if(writeConfirm(0x2203, 0)) { score ++; } // velocity feedforward gain
        if(writeConfirm(0x2204, 2000)) { score ++; } // 
        // VALUES ABOVE 

        // torque control settings
        if(writeConfirm(0x2500, 1)) { reg_0500 = 1; score++; } 
        if(writeConfirm(0x2501, 1)) { reg_0501 = 1; score++; } 
        if(writeConfirm(0x2502, 0)) { reg_0502 = 0; score++; }
        if(writeConfirm(0x2503, 0)) { reg_0503 = 0; score++; }
        if(writeConfirm(0x2510, (Menu1[SETMOTORTORQUE].value))) { reg_0510 = (Menu1[SETMOTORTORQUE].value); score++; } 
        if(writeConfirm(0x2511, -(Menu1[SETMOTORTORQUE].value))) { reg_0511 = -(Menu1[SETMOTORTORQUE].value); score++; } 
       if (score < 12) { error = 7; return false; } // Total score count = 12 if i can count 
      } else { 
        reg_0101 = 2;
        reg_0102 = 1;
        reg_0103 = Menu1[SETMOTORRAMP].value;
        reg_0121 = 30;
        reg_0500 = 1;
        reg_0501 = 1;
      }
      reg_6040 = 1;
      lastCommReceived = millis();
      nextCommType = 0;
      commStatus = COMM_CONNECTED;
      currentStatus = MOTOR_NOT_READY;
      commCounter = 0;
    }
    lastCommSend = millis();
    return false;
  }

  // The only FAST & blocking service we're offering!
  if(motor_setRPM == 0 && motor_lastSetRPM != 0) { 
     if(reg_0502 != 0) { 
      if(writeConfirm(0x2503, 0)) { // Set rotation speed to 0.
        lastCommSend = millis(); 
        reg_0502 = 0;
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }
    motor_lastSetRPM = motor_setRPM;
    nextCommType = 0;
    return false;
  }

  // >>>>>>>> RECEIVE <<<<<<<< 
  int16_t received = receive();
  if(received > 0) {

    // STATUS UPDATING
    commStatus = COMM_CONNECTED;
    if(currentStatus == MOTOR_NOT_CONNECTED) currentStatus = MOTOR_NOT_READY;
    lastCommReceived = millis(); 

    #ifdef DEBUG_MOTORCONTROL
      Serial.print("received!: "), Serial.println(received);
      Serial.print("Commcounter: "), Serial.println(commCounter);
      Serial.print("next comm type: "), Serial.println(nextCommType);

      if(lastRequestType == 0 || lastRequestType > 200) {
        Serial.println("received write succesfull!");
        Serial.print("Addr: "), Serial.print(received, HEX); 
        if(lastRequestType == 0) { Serial.print(" value: "), Serial.println(lastRead[0]); } 
        else { Serial.println(""); } 
      }
    #endif
    // ERROR HANDLING 
    if(error == 2 || error == 1) { 
      error = 0;
      disp_updateRequired = true;
    }

    // >>>>>> SEND SINGLE WRITE
    if(lastRequestType == 0) {  
      switch(received) {
        case 0x6040: { reg_6040 = lastRead[0]; break; } 

        case 0x2103: { reg_0103 = lastRead[0]; break; }

        case 0x2500: { reg_0500 = lastRead[0]; break; }
        case 0x2501: { reg_0501 = lastRead[0]; break; }
        case 0x2502: { reg_0502 = lastRead[0]; break; }
        case 0x2503: { reg_0503 = lastRead[0]; break; }
        case 0x2510: { reg_0510 = lastRead[0]; break; }
        case 0x2511: { reg_0511 = lastRead[0]; break; }
        default: break;
      }
    }

    // >>>>> DATA REQUESTS
    if(lastRequestType > 100 && lastRequestType < 200) { 
      #ifdef DEBUG_MOTORCONTROL 
        #ifndef DEBUG_COMM
        Serial.println("Newdata from read!");
        Serial.print("Addr: "), Serial.print(received, HEX), Serial.print(" value: "), Serial.println(lastRead[0]);
        if(received > 101) Serial.print("Addr: "), Serial.print(received+1, HEX), Serial.print(" value: "), Serial.println(lastRead[1]);
        if(received > 102) Serial.print("Addr: "), Serial.print(received+2, HEX), Serial.print(" value: "), Serial.println(lastRead[2]);  
        if(received > 103) Serial.print("Addr: "), Serial.print(received+3, HEX), Serial.print(" value: "), Serial.println(lastRead[3]);   
        #endif      
      #endif

      if(received == 0x606C) {
        motor_currentRPM = lastRead[1]; //discarding the high word since we dont use it
        disp_updateRequired = true;
      }
      if(received == 0x6077) { 
        motor_currentTorque = lastRead[0];
        newData = true;
      }

      if(received == 0x6041) { 
        currentStatus = changeToStatus(decodeStatusWord(lastRead[0]));
        disp_updateRequired = true;
      }
    }

    // >>>>> Send multiple writes
    if(lastRequestType > 200 && lastRequestType < 205) { //A SEND of max 4 writes -->> 
      // does not work properly on JMC factory registries... registries are not continouous?? reading not either
      // Does work for 32 bit registers like 0x6081 example:
      // if(received == 0x6081) { 
      //   reg_6081 = pending_reg_6081;
      //   reg_6082 = pending_reg_6082;
      // }
    } 

    return true;
  } else newData = false; // We've made one complete round!

  //Determine COMM delay
  uint16_t delay;
  if(motor_setRPM == 0) if(reg_6040!=3 && reg_0503!=0 && reg_0502 != 0 && nextCommType > 0) nextCommType = 0; //We need to SEND
  if (nextCommType == 0) delay = COMM_DELAY_SEND; 
  else if(nextCommType > 0) { 
    if (state == IDLE || state == IDLE_GBW || state == MENU1 || state == MENU2 || state == MENU3) delay = COMM_DELAY_IDLE;
    else delay = COMMINTERVAL;
  }
  if(lastCommSend + delay > millis()) return false;

// >>>>>> MOTOR OFF <<<<<<<<
if(motor_setRPM == 0) { 
    if(reg_0503 != 0) { 
      if(writeSingleRegister(0x2503, 0)) { // Set torq command to 0
        lastCommSend = millis(); 
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_0502 != 0) { 
      if(writeSingleRegister(0x2502, 0)) { // Set rotation speed 0
        lastCommSend = millis(); 
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_6040 != 3) { 
      if(writeSingleRegister(0x6040, 3)) { //SERVO DISABLE
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    // SET ONCE >> 
    if(reg_0103 != Menu1[SETMOTORRAMP].value && state == IDLE || state == IDLE_GBW) { 
      if(writeConfirm(0x2103, Menu1[SETMOTORRAMP].value)) { // Set rigidity LVL (restart required)
        reg_0103 = Menu1[SETMOTORRAMP].value;
        error = 107; // reboot drive error
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_0510 != (Menu1[SETMOTORTORQUE].value)) { 
      if(writeSingleRegister(0x2510, (Menu1[SETMOTORTORQUE].value))) { // Set fw torque limit 
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }
    if(reg_0511 != -(Menu1[SETMOTORTORQUE].value)) { 
      if(writeSingleRegister(0x2511, -(Menu1[SETMOTORTORQUE].value))) { // Set fw torque limit 
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }
    // << SET ONCE
    
    // Check if we've done all commands.
    if(reg_6040 == 3 && reg_0502 == 0 && reg_0503 == 0 && reg_0101 == 2) { 
      if(nextCommType == 0) nextCommType = 1;
    }
  }

  //>>>>> Forward ROTATION <<<<<<<<<
  if(motor_setRPM > 0) { 
    if(reg_0502 != motor_setRPM) { 
      if(writeSingleRegister(0x2502, motor_setRPM)) { // Set fw speed
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_6040 != 0x0F && reg_0502 == motor_setRPM && reg_0503 >= 0) { 
      if(writeSingleRegister(0x6040, 0x0F)) { //SERVO ENABLE
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_0503 != (Menu1[SETMOTORTORQUE].value)) { 
      if(writeSingleRegister(0x2503, (Menu1[SETMOTORTORQUE].value))) { // Set fw torque target
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    //Check if we've done all commands
    if(reg_6040 == 0x0F && reg_0502 == motor_setRPM && reg_0503 == (Menu1[SETMOTORTORQUE].value)) { 
      if(nextCommType == 0) nextCommType = 1; 
    }
  }
  
  // >>>>>>>>> REVERSE ROTATION <<<<<<<<<
  if(motor_setRPM < 0) { 
    if(reg_0502 != -motor_setRPM) { 
      if(writeSingleRegister(0x2502, -motor_setRPM)) { // Set rpm limit
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_6040 != 0x0F && reg_0502 == -motor_setRPM && reg_0503 <= 0) { 
      if(writeSingleRegister(0x6040, 0x0F)) { //SERVO ENABLE
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_0503 != -(Menu1[SETMOTORTORQUE].value)) { 
      if(writeSingleRegister(0x2503, -(Menu1[SETMOTORTORQUE].value))) { // Set rv torque limit 
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    //Check if we've done all commands
    if(reg_6040 == 0x0F && reg_0502 == -motor_setRPM && reg_0503 == -(Menu1[SETMOTORTORQUE].value)) { 
      if(nextCommType == 0) nextCommType = 1; 
    }
  }

  // READING
  if(nextCommType == 1) { 
    nextCommType++;
    readRegister(0x6041, 1); //Read status word every 10
    lastCommSend = millis();
    return false;
  }

  if(nextCommType > 1 && nextCommType < 10) {
    if (nextCommType % 2 == 0) {
      nextCommType++;
      readRegister(0x606C, 1);
      lastCommSend = millis();
    } else {
      nextCommType++;
      readRegister(0x6077, 1);
      lastCommSend = millis();
    }
  }

  if(nextCommType >= 10) { 
    nextCommType = 0;
  }
  return false;
}

bool motorOff() {
    for(int i = 0; i < 10; i++) {
      if(writeConfirm(0x6040, 3)) {
        lastCommSend = millis();
        reg_6040 = 3;
        return true;
      } 
      delay(200);
    }
    return false;
}

void motorSleep() { 
  #ifdef DEBUG_MOTORCONTROL
  Serial.print("Motor sleep..");
  #endif
    for(int i = 0; i < 10; i++) {
      delay(200);
      if(writeConfirm(0x6040, 0)) { //Set control word to 0 = lowest power state possible.
        lastCommSend = millis();
        reg_6040 = 0;
        #ifdef DEBUG_MOTORCONTROL
          Serial.println("Servo drive set to low power state");
        #endif
        return;
      } 
    }
}

void motorReset() { 
  writeConfirm(0x2101, 1); // Reset control mode to speed mode
  writeMultipleConfirm(0x1010, 2, 0, 0); // Set magic byte to 0
  writeConfirm(0x6060, 3); //Set CiA402 to speedmode
  motorSleep(); //Should prevent the necessity of a restart, unsure tho.
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

    if(commStatus != COMM_CONNECTED) { //Can't go here if comm is bad.
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

        if(motor_setRPM < 0) { 
          #ifdef DEBUG_CALIB
            Serial.print("Motor set ERROR <0: "), Serial.println(motor_setRPM);
          #endif
          motor_setRPM = 0;
        }

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

#endif