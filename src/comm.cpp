#include "definitions.h"
#include "HardwareSerial.h"
#include "comm.h"  
#include "main.h"

#ifdef RT_DRIVE
#include "motorcontrol_rt.h"
#endif
#ifdef JMC_DRIVE
#include "motorcontrol_jmc.h"
#endif

unsigned long lastSend;
unsigned long lastReceived = 0;
uint16_t errorCode = 0;
uint8_t request[20] = {0};
uint8_t response[32] = {0};
uint16_t lastRead[20] = {0};
bool responseReceived = 0;
uint8_t lastRequestType = 0;
uint16_t crc = 0;
uint16_t lastRegAddr = 0;

void comm_init() { 
  pinMode(RS485DE, OUTPUT);
  pinMode(RS485RE, OUTPUT);
  Serial2.begin(BAUD_RATE, PARITY, RS485_RXD, RS485_TXD);
  #ifdef DEBUG_COMM
    Serial.println("comm init");
  #endif
}

bool writeConfirm(uint16_t reg_addr, uint16_t value) { 
  while(millis() - lastSend < COMM_DELAY_SEND);
  writeSingleRegister(reg_addr, value);
  delay(COMM_DELAY_RECEIVE + 1);
  bool response = false;
  int16_t received = receive();
  if(received == reg_addr) response = true;
  return response;
}

bool writeMultipleConfirm(uint16_t reg_addr_start, uint16_t reg_count, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4) { 
  while(millis() - lastSend < COMM_DELAY_SEND);
  writeMultipleRegisters(reg_addr_start, reg_count, data1, data2, data3, data4);
  delay(COMM_DELAY_RECEIVE + 1);
  bool response = false;
  #ifdef DEBUG_COMM
    Serial.print("received bytes: "), Serial.println(Serial2.available());
  #endif
  int16_t received = receive();
  if(received == reg_addr_start) response = true;
  return response;
}

int16_t readReturn(uint16_t reg_addr, uint16_t reg_count) { //returns 0xFFFF if unsuccesfull, otherwise returns read value of first reg_addr
  while(millis() - lastSend < COMM_DELAY_SEND);
  readRegister(reg_addr, reg_count); //Send read request
  #ifdef DEBUG_COMM
    Serial.print("requesting: "), Serial.println(reg_addr);
  #endif
  while(millis() - lastSend < COMM_DELAY_RECEIVE);
  if(receive() == reg_addr) { 
    #ifdef DEBUG_COMM
    Serial.print("Readreturn, lastRead0: "), Serial.println(lastRead[0]);
    #endif
    return lastRead[0];
  } else return 0xFFFF;
}

void rs485_send(uint8_t* data, size_t length) {
  #ifdef DEBUG_COMM
  Serial.print("sending: ");
    for(int i = 0; i < length; i++) { 
      Serial.print(data[i], HEX); 
      Serial.print(" ");
    }
  Serial.println("<end>");
  #endif
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

bool writeMultipleRegisters(uint16_t reg_addr_start, uint16_t reg_count,
                           uint16_t data1, uint16_t data2,
                           uint16_t data3, uint16_t data4) {
    if (lastSend + COMM_DELAY_SEND > millis()) return false;
    if (reg_count < 1 || reg_count > 4) return false;

    request[0] = SLAVE_ID;
    request[1] = FUNC_WRITE_MULTIPLE_REGISTERS;
    request[2] = reg_addr_start >> 8;
    request[3] = reg_addr_start & 0xFF;
    request[4] = reg_count >> 8;
    request[5] = reg_count & 0xFF;
    request[6] = reg_count * 2;

    uint16_t data[4] = { data1, data2, data3, data4 };
    for (uint8_t i = 0; i < reg_count; ++i) {
        request[7 + i * 2] = data[i] >> 8;
        request[8 + i * 2] = data[i] & 0xFF;
    }

    uint8_t msgLen = 7 + reg_count * 2;
    crc = modbus_crc16(request, msgLen);
    request[msgLen]     = crc & 0xFF;
    request[msgLen + 1] = crc >> 8;

    while (Serial2.available()) Serial2.read();
    rs485_send(request, msgLen + 2);

    lastSend = millis();
    lastRequestType = 200 + reg_count;
    return true;
}

bool receiveError(uint8_t msgLength) {
  // Serial2.readBytes(response, 5);

  //DEBUG COMM
  #ifdef DEBUG_COMM
  Serial.println("Fault code received!");
    Serial.print("Last req type: "), Serial.println(lastRequestType);
    Serial.print("Message: ");
    for(int i = 0; i < 5; i++) { 
      Serial.print(response[i], HEX);
      Serial.print(" ");
    }
    Serial.print(" < end > "), Serial.print("bytes: "), Serial.println(5);
  #endif

  if(msgLength == 5 && response[1] >= 0x80 && response[1] <= 0x90) { //error frame
    uint16_t crc_resp = response[3] | (response[4] << 8);
    if(crc_resp != modbus_crc16(response, 3)) return 0;

    errorCode = response[2];
    error = 3; 
    Serial.print("error received: ");
    Serial.println(errorCode);

    return 0x603F; //Error code cia402
  }
  return 0;
}

int16_t receive() {  // Returns 0 when nothing is read. returns the read register when succesfull receive(). 
  if(lastSend + COMM_DELAY_RECEIVE > millis()) return 0; //exit if not expecting a complete message yet, faster and safer than checking serial.available();

  int len;
  len = Serial2.available();

  Serial2.readBytes(response, len);
  memset(lastRead, 0, sizeof(lastRead));

  //DEBUG COMM
  #ifdef DEBUG_COMM
    if(len > 2) {
      Serial.print("Last req type: "), Serial.println(lastRequestType);
      Serial.print("Message: ");
      for(int i = 0; i < len; i++) { 
        Serial.print(response[i], HEX);
        Serial.print(" ");
      }
      Serial.print(" < end > "), Serial.print("bytes: "), Serial.println(len);
    }
  #endif

  if(len == 5) return(receiveError(5)); //If we're between received and new send (delay)
  if(lastRequestType == 0 && len < 8) return 0;
  if(lastRequestType > 100 && lastRequestType < 200 && len < 7) return 0;
  if(lastRequestType > 200 && len < 8) return 0;
  // READ
  

  // >>>>>>>>>>>> Last comm request was a writeSingleReg
  if(lastRequestType == 0) { 
    uint16_t crc_resp = response[6] | (response[7] << 8);
    if (modbus_crc16(response, 6) != crc_resp) {
      #ifdef DEBUG_COMM
        Serial.println("CRC check failed0");
      #endif
      return false;
    } else { 
      #ifdef DEBUG_COMM
        Serial.println("CRC OK"); 
      #endif
      lastReceived = millis();
    }

    if(crc == crc_resp) { //SEND CRC matches received -> messages match -> Succes!
      commCounter--;
      lastRead[0] = (response[4] << 8) | response[5];
      return (response[2] << 8) | response[3];
    };

    #ifdef DEBUG_COMM
      Serial.println("Received data unexpected"); //Returned value is different that the set value
    #endif
    return false;
  }

  // >>>>>>>>>>>> Last send type was a data request (101 = 1 register)
  if (lastRequestType > 100 && lastRequestType < 200) {  

    // Validate CRC
    uint16_t crc_resp = response[len - 2] | (response[len - 1] << 8);
    if (modbus_crc16(response, len - 2) != crc_resp) {
      #ifdef DEBUG_COMM
        Serial.println("CRC failed 100+");
      #endif
      return 0;
    }

    // VALIDATE MSG 
    if (response[0] != SLAVE_ID || response[1] != FUNC_READ_HOLDING_REGISTERS) {
      #ifdef DEBUG_COMM
        Serial.print("Error: SLAVE ADDR:");
        Serial.println(response[0], HEX);
        Serial.print("Error: FUNCTION CODE");
        Serial.println(response[1], HEX);
      #endif
      return 0;
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
    if((lastRequestType-100)*2 != byteCount) lastRequestType = byteCount/2 + 100;
    }
    commCounter--;
    return lastRegAddr;
  }

  //  >>>>>>>>>>>> receive multiplewrites
  if(lastRequestType > 200) { 
    // Validate CRC
    uint16_t crc_resp = response[len - 2] | (response[len - 1] << 8);
    if (modbus_crc16(response, len - 2) != crc_resp) {
      #ifdef DEBUG_COMM
        Serial.println("Error: CRC failed>200");
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

    // VALIDATE REG ADDR
    if(response[2] == request[2] && response[3] == request[3] && response[4] == request[4] && response[5] == request[5]) { 
      commCounter--;
      return (response[2] << 8) | response[3];
    } else {
        #ifdef DEBUG_COMM
          Serial.println("Response != request");
          Serial.print("Reg Requested: "), Serial.print(lastRegAddr), Serial.print("Reg Received: "), Serial.println((response[2] << 8) | response[3]);
      #endif 
    }
    return 0;
  }
  return 0;
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
  lastRegAddr = reg_addr;
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