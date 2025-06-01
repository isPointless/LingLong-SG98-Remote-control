#include "definitions.h"

#ifndef IO_H
#define IO_H

void io_init();
bool check_io();
void ledAction(uint8_t type);

int16_t encoder_change();

void do_io();

bool START_BUTTON();
bool ENC_BUTTON();

bool pressedLong();

#endif