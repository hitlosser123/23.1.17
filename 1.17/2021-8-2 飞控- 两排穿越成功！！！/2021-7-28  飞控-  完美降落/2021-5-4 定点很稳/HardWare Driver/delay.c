#include "delay.h"

void delay_us(uint32_t nus)
{
	unsigned int i=40;
	i*=nus;
	while(i--);
}
void delay_ms(uint32_t nms)
{
	delay_us(nms*1000);
}

