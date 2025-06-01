#include "definitions.h"

#ifndef pdata_h
#define pdata_h

#include <Preferences.h>
void pdata_init();
void pdata_write(uint8_t what, uint8_t specific);
void pdata_read();

#endif