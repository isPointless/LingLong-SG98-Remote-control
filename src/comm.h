#include "definitions.h"

#ifndef comm_h
#define comm_h

#define SLAVE_ID 0x01
#define FUNC_READ_HOLDING_REGISTERS 0x03
#define FUNC_WRITE_SINGLE_REGISTER 0x06
#define FUNC_WRITE_MULTIPLE_REGISTERS 0x10

void comm_init(); //init RS485 communication

bool writeConfirm(uint16_t reg_addr, uint16_t value);
uint16_t readReturn(uint16_t reg_addr, uint16_t reg_count);

bool writeSingleRegister(uint16_t reg_addr, uint16_t value);
bool readRegister(uint16_t reg_addr, uint16_t reg_count);

extern bool receive(); 

extern uint16_t lastRead[8]; 
extern uint8_t lastRequestType; //0 = writeSingle, 101 = readSingleRegister, 102 = read two registers etc
extern uint16_t errorCode; 


uint16_t modbus_crc16(uint8_t* buf, int length);
bool receiveError(uint8_t msgLength); 

inline void setTXmode() { 
  digitalWrite(RS485DE, HIGH); 
  digitalWrite(RS485RE, HIGH); 
} // check if theyre tied together or not??
inline void setRXmode() { 
  digitalWrite(RS485DE, LOW); 
  digitalWrite(RS485RE, LOW); 
}

#endif

