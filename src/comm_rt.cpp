#include "HardwareSerial.h"
#include "definitions.h"
#include "comm_rt.h"  
#include "main.h"

#include "comm_rt.h"
#include "motorcontrol_rt.h"

unsigned long lastSend;
unsigned long lastReceived = 0;
uint16_t errorCode = 0;
uint8_t request[20] = {0};
uint8_t response[32] = {0};
uint16_t lastRead[20] = {0};
bool responseReceived = 0;
uint8_t lastRequestType = 0;
uint16_t crc = 0;

void comm_init() { 
  pinMode(RS485DE, OUTPUT);
  pinMode(RS485RE, OUTPUT);
  Serial2.begin(BAUD_RATE, PARITY, RS485_RXD, RS485_TXD);
  #ifdef DEBUG_COMM
    Serial.println("comm init");
  #endif
}

bool writeConfirm(uint16_t reg_addr, uint16_t value) { 
  while(!writeSingleRegister(reg_addr, value))
  delay(COMM_DELAY_RECEIVE);
  return receive();
}

bool writeMultipleConfirm(uint16_t reg_addr_start, uint16_t reg_count, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4) { 
  while(!writeMultipleRegisters(reg_addr_start, reg_count, data1, data2, data3, data4))
  delay(COMM_DELAY_RECEIVE);
  return receive();
}

uint16_t readReturn(uint16_t reg_addr, uint16_t reg_count) { //returns -1 if unsuccesfull, otherwise returns read value of first reg_addr
  while(!readRegister(reg_addr, reg_count)) 
  delay(COMM_DELAY_RECEIVE);
  if(receive() == true) { 
    return lastRead[0];
  } else return -1;
}

void rs485_send(uint8_t* data, size_t length) {

  setTXmode();
  delayMicroseconds(500);
  Serial2.write(data, length);
  Serial2.flush();
  delayMicroseconds(500);
  setRXmode();
  lastSend = millis();
  commCounter++;
}

bool writeSingleRegister(uint16_t reg_addr, uint16_t value) { //returns true when succesfully SEND
  if(lastSend + COMM_DELAY_SEND > millis()) { return false; }
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

  lastSend = millis();
  lastRequestType = 0;
  return true;
}

bool writeMultipleRegisters(uint16_t reg_addr_start, uint16_t reg_count, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4) { //returns true when succesfully SEND
  if(lastSend + COMM_DELAY_SEND > millis()) return false;
  int length;
  request[0] = SLAVE_ID;
  request[1] = FUNC_WRITE_MULTIPLE_REGISTERS;
  request[2] = reg_addr_start>> 8;
  request[3] = reg_addr_start & 0xFF;
  request[4] = reg_count>> 8;
  request[5] = reg_count & 0xFF;
  request[6] = uint8_t(reg_count * 2);
  request[7] = data1 >> 8;
  request[8] = data1 & 0xFF;
  if(reg_count == 1) { 
    request[9] = crc & 0xFF;
    request[10] = crc >> 8;
    length = 11;
  }
  if(reg_count == 2) { 
    request[9] = data2 >> 8;
    request[10] = data2 & 0xFF;
    crc = modbus_crc16(request, 11);
    request[11] = crc & 0xFF;
    request[12] = crc >> 8;
    length = 13;
  }
  if(reg_count == 3) { 
    request[9] = data2 >> 8;
    request[10] = data2 & 0xFF;
    request[11] = data3 >> 8;
    request[12] = data3 & 0xFF;
    crc = modbus_crc16(request, 13);
    request[13] = crc & 0xFF;
    request[14] = crc >> 8;
    length = 15;
  }
   if(reg_count == 4) { 
    request[9] = data2 >> 8;
    request[10] = data2 & 0xFF;
    request[11] = data3 >> 8;
    request[12] = data3 & 0xFF;
    request[13] = data4 >> 8;
    request[14] = data4 & 0xFF;
    crc = modbus_crc16(request, 15);
    request[15] = crc & 0xFF;
    request[16] = crc >> 8;
    length = 16;
  }

  while (Serial2.available()) Serial2.read();
  rs485_send(request, length);

  lastRequestType = 200 + reg_count;
  return true;
}

bool receiveError(uint8_t msgLength) {
  Serial2.readBytes(response, 5);
  if(msgLength == 5 && response[1] >= 0x80 && response[1] <= 0x90) { //error frame
    uint16_t crc_resp = response[3] | (response[4] << 8);
    if(crc_resp != modbus_crc16(response, 5)) return false;

    errorCode = response[2];
    error = 3; 
    Serial.print("error received from: ");
    Serial.println(errorCode);

    return true;
  }
  return false;
}

bool receive() { 
  if(lastSend + COMM_DELAY_RECEIVE > millis()) return false; //exit if not expecting a complete message yet, simpler than checking serial.available();
  if(Serial2.available() == 5) receiveError(5); //If we're between received and new send (delay)
  if(lastRequestType == 0 && Serial2.available() < 8) return false;
  if(lastRequestType == 104 && Serial2.available() < 13) return false;
  if(lastRequestType > 200 && Serial2.available() < 8) return false;

  int len;
  
  len = Serial2.available();
  Serial2.readBytes(response, len);
  if(len == 5) return receiveError(len);

  #ifdef DEBUG_COMM
    Serial.println("Received bytecount: " + len);
    for(int i = 0; i < len; i++) { 
      Serial.print(response[i], HEX);
      Serial.print(" ");
    }
    Serial.println(" < end of message > ");
    Serial.println("last req type: " + lastRequestType);
  #endif
  
  // Last comm request was a writeSingleReg
  if(lastRequestType == 0) { 
    if (len < 8) {
      #ifdef DEBUG_COMM
        Serial.println("Response timeout or incomplete");
      #endif
      return false;
    }

    uint16_t crc_resp = response[6] | (response[7] << 8);
    if (modbus_crc16(response, 6) != crc_resp) {
      #ifdef DEBUG_COMM
        Serial.println("CRC check failed");
      #endif
      return false;
    } else { 
      #ifdef DEBUG_COMM
        Serial.println("CRC OK"); 
      #endif
      lastReceived = millis();
    }

    if(crc == crc_resp) { //SEND CRC matches received -> messages match -> Succes!
      return true;
    };

    #ifdef DEBUG_COMM
      Serial.println("Received data unexpected"); //Returned value is different that the set value
    #endif
    return false;
  }

  //receive multiplewrites
  if(lastRequestType > 200) { 
    if (len < 8) {
      #ifdef DEBUG_COMM
        Serial.println("200: No or incomplete response");
      #endif
      return false;
    }

    // Validate CRC
    uint16_t crc_resp = response[len - 2] | (response[len - 1] << 8);
    if (modbus_crc16(response, len - 2) != crc_resp) {
      #ifdef DEBUG_COMM
        Serial.println("Error: CRC failed");
      #endif
      return false;
    } else {
      #ifdef DEBUG_COMM
      Serial.println("CRC succes!");
      #endif
      lastReceived = millis();
    }

    // VALIDATE MSG 
    if (response[0] != SLAVE_ID || response[1] != FUNC_WRITE_MULTIPLE_REGISTERS) {
      #ifdef DEBUG_COMM
        Serial.print("Error: SLAVE ADDR:");
        Serial.println(response[0], HEX);
        Serial.print("Error: FUNCTION CODE");
        Serial.println(response[1], HEX);
      #endif
      return false;
    }

    if(response[2] == request[2]) return true;
    else { 
        Serial.println("Response != request");
        #ifdef DEBUG_COMM
          Serial.print("ERROR! Request 2: ");
          Serial.print(request[2], HEX);
          ;Serial.print(" Response 2: ");
          Serial.println(response[2], HEX);
          Serial.print("ERROR! Request 3: "); 
          Serial.print(request[3], HEX); 
          Serial.print(" Response 3: ");
          Serial.println(response[3], HEX);
          Serial.print("ERROR! Request 5: ");
          Serial.print(request[5], HEX); 
          Serial.print(" Response 5: ");
          Serial.println(response[5], HEX);
      #endif
    }
    return false;
  }

  // Last send type was a data request (101 = 1 register)
  if ( lastRequestType > 100 && lastRequestType < 200) {  

    // Validate CRC
    uint16_t crc_resp = response[len - 2] | (response[len - 1] << 8);
    if (modbus_crc16(response, len - 2) != crc_resp) {
      #ifdef DEBUG_COMM
        Serial.println("Error: CRC failed");
      #endif
      return false;
    }

    // VALIDATE MSG 
    if (response[0] != SLAVE_ID || response[1] != FUNC_READ_HOLDING_REGISTERS) {
      #ifdef DEBUG_COMM
        Serial.print("Error: SLAVE ADDR:");
        Serial.println(response[0], HEX);
        Serial.print("Error: FUNCTION CODE");
        Serial.println(response[1], HEX);
      #endif
      return false;
    }

    // SAVE DATA FROM RESPONSE
    memset(lastRead, 0, sizeof(lastRead));
    uint8_t byteCount = response[2];
    #ifdef DEBUG_COMM
      Serial.print("Registers read: ");
      Serial.println(byteCount / 2);
    #endif
    for (int i = 0; i < byteCount; i += 2) {
      lastRead[i/2] = (response[3 + i] << 8) | response[4 + i];
      #ifdef DEBUG_COMM
        Serial.print("Lastread #" + i/2), Serial.print(" : "), Serial.println(lastRead[i/2]);
      #endif
    }
    return true;
  }
  return false;
}

bool readRegister(uint16_t reg_addr, uint16_t reg_count) { //returns true when succesfully SEND
  if(lastSend + COMM_DELAY_SEND > millis()) { return false; }
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
