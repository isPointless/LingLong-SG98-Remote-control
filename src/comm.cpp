#include "definitions.h"
#include "comm.h"  
#include "main.h"

unsigned long lastSend;
uint16_t errorCode = 0;
uint8_t request[8] = {0};
uint8_t response[32] = {0};
uint16_t lastRead[8] = {0};
bool responseReceived = 0;
uint8_t lastRequestType = 0;
uint16_t crc = 0;

void comm_init() { 
  pinMode(RXD, OUTPUT);
  pinMode(TXD, OUTPUT);
  Serial2.begin(BAUD_RATE, PARITY, RXD, TXD);
  #ifdef DEBUG
    serial.println("comm init");
  #endif
}

bool writeConfirm(uint16_t reg_addr, uint16_t value) { 
  while(!writeSingleRegister(reg_addr, value))
  delay(COMM_DELAY);
  return receive();
}

uint16_t readReturn(uint16_t reg_addr, uint16_t reg_count) { //returns -1 if unsuccesfull, otherwise returns read value of first reg_addr
  while(!readRegister(reg_addr, reg_count)) 
  delay(COMM_DELAY);
  if(receive() == true) { 
    return lastRead[0];
  } else return -1;
}

void rs485_send(uint8_t* data, size_t length) {
  setTXmode();
  delayMicroseconds(400);
  Serial2.write(data, length);
  Serial2.flush();
  delayMicroseconds(500);
  setRXmode();
  lastSend = millis();
}

bool writeSingleRegister(uint16_t reg_addr, uint16_t value) {
  if(lastSend + COMMINTERVAL > millis()) { return false; }
  request[0] = SLAVE_ID;
  request[1] = FUNC_WRITE_SINGLE_REGISTER;
  request[2] = reg_addr >> 8;
  request[3] = reg_addr & 0xFF;
  request[4] = value >> 8;
  request[5] = value & 0xFF;
  crc = modbus_crc16(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;

  while (Serial2.available()) Serial2.read();
  rs485_send(request, 8);

  lastRequestType = 0;
  return true;
}

bool receiveError(uint8_t msgLength) {
  if(msgLength == 5 && response[1] >= 0x80 && response[1] <= 0x90) { //error frame
    uint16_t crc_resp = response[4] | (response[5] << 8);
    errorCode = response[2] | (response[3] << 8);
    error = 3; 
    #ifdef DEBUG
      Serial.print("error received: ");
      Serial.println(errorCode);
    #endif
    return true;
  }
  return false;
}

bool receive() { 
  uint8_t len; 

  if(lastSend + COMM_DELAY > millis()) return false; //exit if not expecting a complete message yet, simpler than checking serial.available();
  
  // Last comm request was a writeSingleReg
  if(lastRequestType == 0) { 
    len = Serial2.readBytes(response, 8);
    if (len < 8) {
      if(receiveError(len) == true) { 
        return true;
      }
      #ifdef DEBUG
        Serial.println("Response timeout or incomplete");
      #endif
      return false;
    }

    uint16_t crc_resp = response[6] | (response[7] << 8);
    if (modbus_crc16(response, 6) != crc_resp) {
      #ifdef DEBUG
        Serial.println("CRC check failed");
      #endif
      return false;
    }

    if(crc == crc_resp) { //CRC matches .. sorta means messages match -> meaning the command came in properly
      return true;
    };

    #ifdef DEBUG
      Serial.println("Received data unexpected"); //Returned value is different that the set value
    #endif
    return false;
  }
  

  // Last send type was a data request (101 = 1 register)
  if ( lastRequestType > 100) {  
    Serial2.readBytes(response, len);
    if (len < (5 + 2*(lastSend-100))) {
      if(receiveError(len) == true) { 
        return true;
      }
      #ifdef DEBUG
        Serial.println("Error: No or incomplete response");
      #endif
      return false;
    }

    // Validate CRC
    uint16_t crc_resp = response[len - 2] | (response[len - 1] << 8);
    if (modbus_crc16(response, len - 2) != crc_resp) {
      #ifdef DEBUG
        Serial.println("Error: CRC failed");
      #endif
      return false;
    }

    // VALIDATE MSG 
    if (response[0] != SLAVE_ID || response[1] != FUNC_READ_HOLDING_REGISTERS) {
      #ifdef DEBUG
        Serial.print("Error: SLAVE ADDR:");
        Serial.println(response[0], HEX);
        Serial.print("Error: FUNCTION CODE");
        Serial.println(response[1], HEX);
      #endif
      return false;
    }

    // SAVE DATA FROM RESPONSE
    lastRead[8] = {0};
    uint8_t byteCount = response[2];
    #ifdef DEBUG
      Serial.print("Registers read: ");
      Serial.println(byteCount / 2);
    #endif
    for (int i = 0; i < byteCount; i += 2) {
      lastRead[i/2] = (response[3 + i] << 8) | response[4 + i];
      #ifdef DEBUG
        Serial.print("Register ");
        Serial.print(reg_addr + (i / 2));
        Serial.print(": ");
        Serial.println(lastRead[i/2]);
      #endif
      
    }
    return true;
  }
  return false;
}

bool readRegister(uint16_t reg_addr, uint16_t reg_count) { //Returns the value of the first register, next registers can be obtained with lastRead[1] and up.
  if(lastSend + COMMINTERVAL > millis()) { return false; }
  request[0] = SLAVE_ID;
  request[1] = FUNC_READ_HOLDING_REGISTERS;
  request[2] = reg_addr >> 8;
  request[3] = reg_addr & 0xFF;
  request[4] = reg_count >> 8;
  request[5] = reg_count & 0xFF;

  uint16_t crc = modbus_crc16(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;

  // Clear previous data
  while (Serial2.available()) Serial2.read();

  rs485_send(request, 8);
  lastRequestType = 100 + reg_count;
  return true;
}

// ---- MODBUS CRC16 ----
uint16_t modbus_crc16(uint8_t* buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

