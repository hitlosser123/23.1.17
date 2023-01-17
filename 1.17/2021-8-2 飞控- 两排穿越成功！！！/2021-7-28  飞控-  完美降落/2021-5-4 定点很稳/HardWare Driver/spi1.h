#ifndef __SPI1_H
#define __SPI1_H
#include "includes.h"

 	    													  
void SPI1_Init(void);			 //初始化SPI口
u8 SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节


u8 SPI1_Read_Byte(u8 reg,uint32_t CS_PORT, uint8_t CS_IO);
u8 SPI1_Write_Byte(u8 reg,u8 data,uint32_t CS_PORT, uint8_t CS_IO);
u8 SPI1_Read_Len(u8 reg,u8 len,u8 *buf,uint32_t CS_PORT, uint8_t CS_IO);


#endif

