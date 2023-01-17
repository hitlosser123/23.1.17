#ifndef __DEBUGERTASK_H
#define __DEBUGERTASK_H	
#include "includes.h"


void DebugerTask(void *pvParameters);
void UartRxDecodeTask(void *pvParameters);

void Send_Data_WDP(void);
extern void imu_Get_One_Byte(unsigned char datax);

#endif

