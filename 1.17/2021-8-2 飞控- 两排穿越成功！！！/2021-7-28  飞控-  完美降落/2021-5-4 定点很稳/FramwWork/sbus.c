#include "sbus.h"
//  4ms data    10.4ms none
unsigned short sbus_channel[16];

void sbus_decode(unsigned char buffer[24]){
	
	sbus_channel[0]  = ((buffer[1]    |buffer[2]<<8)                      & 0x07FF);
	sbus_channel[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
	sbus_channel[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF);
	sbus_channel[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
	sbus_channel[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
	sbus_channel[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
	sbus_channel[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
	sbus_channel[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
	sbus_channel[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
	sbus_channel[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
	sbus_channel[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
	sbus_channel[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
	sbus_channel[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
	sbus_channel[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
	sbus_channel[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
	sbus_channel[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);
	
	if(sbus_channel[0]>10&&sbus_channel[0]<2000)
		sbus_channel[0] = sbus_channel[0];
	else
		sbus_channel[0] = 1024;
	
	if(sbus_channel[1]>10&&sbus_channel[1]<2000)
		sbus_channel[1] = sbus_channel[1];
	else
		sbus_channel[1] = 1024;
	
	if(sbus_channel[2]>10&&sbus_channel[2]<2000)
		sbus_channel[2] = sbus_channel[2];
	else
		sbus_channel[2] = 1024;
	
	if(sbus_channel[3]>10&&sbus_channel[3]<2000)
		sbus_channel[3] = sbus_channel[3];
	else
		sbus_channel[3] = 1024;
}
