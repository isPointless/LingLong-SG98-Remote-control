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

COMM_STATUS commStatus = COMM_DISCONNECTED;
SYSTEM_STATUS currentStatus = MOTOR_NOT_CONNECTED;

int16_t calibrateArray[SIZEOFCALIBRATEARRAY] = {0};

int16_t reg_6040 = -1; //Control word
int16_t reg_6060 = -1; //Operation mode.. 3=speed+torque
int16_t reg_6000 = -1; //Prob some comm flag or error reset
int16_t reg_6072 = -1; //max torque probably!
int16_t reg_6081 = -1; //RPM
int16_t reg_6082 = -1; //No idea..?

int16_t pending_reg_6081 = 0xFFFF;
int16_t pending_reg_6082 = 0xFFFF;

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
  reg_6060 = -1; //Operation mode.. 3=speed+torque
  reg_6000 = -1; //Prob some comm flag or error reset
  reg_6072 = -1; //max torque probably!
  reg_6081 = -1; //RPM
  reg_6082 = -1; //No idea..?
  commStatus = COMM_DISCONNECTED;
  currentStatus = MOTOR_NOT_CONNECTED;
}

// bool motor_param_set() { 
//   int score = 0;
//   if(writeConfirm(0x6072, 1000)) score++;
//   if(writeConfirm())
// }

bool do_comm() { // can be called continously and will update all values. Returns true when new data packet is available.
    /* COMM TYPES
    0 = send RPM
    1+ = READ STATUS, RPM, TORQUE (1300, 1301, 1302, 1303)
    */
  static unsigned long lastCommSend;
  static unsigned long lastCommReceived;
  // static uint8_t lastCommType;
  static uint8_t nextCommType;
  static int16_t motor_lastSetRPM;

  //Disconnect flow
  if(lastCommReceived + 2*COMM_DELAY_IDLE < millis() && lastCommReceived > 0) { 
    #ifndef DEBUG
      error = 2;
    #endif
    commStatus = COMM_DISCONNECTED;
    currentStatus = MOTOR_NOT_CONNECTED;
    disp_updateRequired = true;
    reg_6040 = -1; //Control word
    reg_6060 = -1; //Operation mode.. 3=speed+torque
    reg_6000 = -1; //Prob some comm flag or error reset
    reg_6072 = -1; //max torque probably!
    reg_6081 = -1; //RPM
    reg_6082 = -1; //No idea..?
  }

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
  int16_t torquePercent = (1000 * motor_currentTorque / Menu1[SETMOTORTORQUE].value); //A value 
  if(motor_setRPM != 0 && torquePercent > 90 && abs(motor_currentRPM) < rpm_scalar) { 
    motor_setRPM = 0;
    state = IDLE; 
    error = 6;
  }

  // First communication when disconnected
  if(commStatus == COMM_DISCONNECTED) { 
    if(writeConfirm(0x6040, 1)) {
      #ifdef DEBUG_MOTORCONTROL
        Serial.println("We're connected!");
      #endif
      reg_6040 = 1;
      reg_6060 = -1; //Operation mode.. 1 = speed
      reg_6000 = -1; //Prob some comm flag or error reset
      reg_6072 = -1; //max torque (uint)
      reg_6081 = -1; //RPM (uint 32) high word
      reg_6082 = -1; //low word RPM, never anything but 0 when not going over 16k rpm 
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
    if(reg_6040 != 3) { 
      if(writeConfirm(0x6040, 3)) { 
        reg_6040 = 3;
      }
    }
    motor_lastSetRPM = motor_setRPM;
    nextCommType = 0;
    return false;
  }

  // >>>>>>>> RECEIVE <<<<<<<< 
  int16_t received = receive();
  if(received > 0) {
    #ifdef DEBUG_MOTORCONTROL
      Serial.print("received!: "), Serial.println(received);
      Serial.print("Commcounter: "), Serial.println(commCounter);
      Serial.print("next comm type: "), Serial.println(nextCommType); 
    #endif
    // ERROR HANDLING 
    if(error == 2 || error == 1) { 
      error = 0;
      disp_updateRequired = true;
    }

    // DEBUG 
    #ifdef DEBUG_MOTORCONTROL
      if(lastRequestType == 0 || lastRequestType > 200) {
        Serial.println("received succesfull!");
        Serial.print("reg 0x6040: "), Serial.println(reg_6040);
        Serial.print("reg 0x6060: "), Serial.println(reg_6060);
        Serial.print("reg 0x6072: "), Serial.println(reg_6072);
        Serial.print("reg 0x6081: "), Serial.println(reg_6081);
        Serial.print("reg 0x6082: "), Serial.println(reg_6082);
      }
    #endif

    // STATUS UPDATING
    commStatus = COMM_CONNECTED;
    if(currentStatus == MOTOR_NOT_CONNECTED) currentStatus = MOTOR_NOT_READY;
    lastCommReceived = millis(); 

    // >>>>>> SEND SINGLE WRITE
    if(lastRequestType == 0) {  
      switch(received) {
        case 0x6040: reg_6040 = lastRead[0];
        case 0x6060: reg_6060 = lastRead[0];
        case 0x6072: reg_6072 = lastRead[0];
        case 0x6081: reg_6081 = lastRead[0];
        case 0x6082: reg_6082 = lastRead[0];
      }
    }

    // >>>>> DATA REQUESTS
    if(lastRequestType > 100 && lastRequestType < 200) { 
      newData = true;
      #ifdef DEBUG_MOTORCONTROL
        Serial.println("Newdata!");
      #endif
      if(received == 0x606C && lastRequestType == 101) { // A request for status, speed & torque
          motor_currentRPM = int16_t(lastRead[0] | (lastRead[1] << 8));
      }
      if(received == 0x6077) { 
        motor_currentTorque = lastRead[0];
      }
      if(received == 6041) { 
        currentStatus = changeToStatus(decodeStatusWord(lastRead[0]));
      }
    }

    // >>>>> Send multiple writes
    if(lastRequestType > 200 && lastRequestType < 205) { //A SEND of max 4 writes
      if(received == 0x6081 && lastRequestType == 202) { 
        reg_6081 = pending_reg_6081;
        reg_6082 = pending_reg_6082;
      }
    } 

    return true;
  } else newData = false; // We've made one complete round!

  //Determine COMM delay
  uint16_t delay;
  if(motor_setRPM == 0) if(reg_6040!=3 && reg_6060!=3 && reg_6000!=1 && reg_6081!=0 && nextCommType > 0) nextCommType = 0; //We need to SEND

  if (nextCommType == 0) delay = COMM_DELAY_SEND; 
  if(nextCommType > 0) { 
    if (state == IDLE || state == IDLE_GBW || state == MENU1 || state == MENU2 || state == MENU3) delay = COMM_DELAY_IDLE;
    else delay = COMMINTERVAL;
  }

  if(lastCommSend + delay > millis()) return false;

// >>>>>> MOTOR OFF <<<<<<<<
if(motor_setRPM == 0) { 
    if(reg_6040 != 3) { 
      if(writeSingleRegister(0x6040, 3)) { //SET CONTROL TO READY (NOT ENABLED)
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }
    if(reg_6060 != 3) { 
      if(writeSingleRegister(6060, 1)) { //SET MODE TO SPEED/TORQUE
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_6072 != Menu1[SETMOTORTORQUE].value) { 
      if(writeSingleRegister(0x6072, Menu1[SETMOTORTORQUE].value)) {//MOTOR TORQUE SET
        lastCommSend = millis(); 
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_6000 != 1) { 
      if(writeSingleRegister(0x6000, 1)) { //Some random comm message i dont understand
        lastCommSend = millis(); 
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }
    
    // Check if we've done all commands.
    if(reg_6040 == 3 && reg_6060 == 3 && reg_6000 == 1 && reg_6072 == Menu1[SETMOTORTORQUE].value) { 
      if(nextCommType == 0) nextCommType = 1;
    }
}

  //>>>>> Forward ROTATION <<<<<<<<<
  if(motor_setRPM > 0) { 
    if(reg_6081 != motor_setRPM || reg_6082 != 0) { 
      if(writeMultipleRegisters(0x6081, 2, motor_setRPM, 0)) { // Set rotation speed, this is actually a 32 bit value, but the second value is always 0 unless rpm > 16bit int.
        lastCommSend = millis(); 
        pending_reg_6081 = motor_setRPM;
        pending_reg_6082 = 0;
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_6072 != Menu1[SETMOTORTORQUE].value) { 
      if(writeSingleRegister(0x6072, Menu1[SETMOTORTORQUE].value)) {// MOTOR TORQUE SET
        lastCommSend = millis(); 
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    if(reg_6040 != 0x0F) { 
      if(writeSingleRegister(0x6040, 0x0F)) { //SERVO ENABLE
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }
    //Check if we've done all commands
    if(reg_6040 == 0x0F && reg_6072 == Menu1[SETMOTORTORQUE].value && reg_6081 == motor_setRPM && reg_6082 == 0) { 
      if(nextCommType == 0) nextCommType = 1; 
    }
  }
  
  // >>>>>>>>> REVERSE ROTATION <<<<<<<<<
  if(motor_setRPM < 0) { 
    if(reg_6081 != -motor_setRPM || reg_6082 != 0) { 
      if(writeMultipleRegisters(0x6081, 2, -motor_setRPM, 0)) { // Set rotation speed
        lastCommSend = millis(); 
        pending_reg_6081 = -motor_setRPM;
        pending_reg_6082 = 1;
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }
    if(reg_6072 != Menu1[SETMOTORTORQUE].value) { 
      if(writeSingleRegister(0x6072, Menu1[SETMOTORTORQUE].value)) {// MOTOR TORQUE SET
        lastCommSend = millis(); 
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }
    if(reg_6040 != 0x0F) { 
      if(writeSingleRegister(0x6040, 0x0F)) { //SERVO ENABLE
        lastCommSend = millis();
        nextCommType = 0;
        motor_lastSetRPM = motor_setRPM;
        return false;
      }
    }

    //Check if we've done all commands
    if(reg_6040 == 0x0F && reg_6072 == Menu1[SETMOTORTORQUE].value && reg_6081 == -motor_setRPM && reg_6082 == 1) { 
      if(nextCommType == 0) nextCommType = 1; 
    }
  }

  //VERIFY PARAMETER SET (DEBUG ONLY)
  #ifdef DEBUG_MOTORCONTROL

  #endif

  // READING
  if(nextCommType == 1) { 
    nextCommType = 1;
    readRegister(0x6041, 1); //Read status word every 10
    lastCommSend = millis();
    return false;
  }

  if(nextCommType > 1) { 
    nextCommType++;
    readRegister(2114, 3);
    lastCommSend = millis();
    return false;
  }

  if(nextCommType > 10) { 
    nextCommType = 1;
  }

  return false;
}

bool motorOff() {
    for(int i = 0; i < 10; i++) {
      if(writeConfirm(0x6040, 3)) { //SET SPEED LIMIT FORWARD = 0
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
      if(writeConfirm(0x6040, 1)) { //Set control word to 0 = lowest power state possible.
        reg_6040 = 1;
        #ifdef DEBUG_MOTORCONTROL
          Serial.println("Servo drive set to low power state");
        #endif
        return;
      } 
    }
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