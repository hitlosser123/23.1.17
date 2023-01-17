#ifndef __SBUS_H
#define __SBUS_H	 
#include "includes.h"

extern unsigned short sbus_channel[16];

void sbus_decode(unsigned char buffer[24]);

#endif
