#ifndef __REMOTERTASK_H
#define __REMOTERTASK_H	 
#include "config&param.h"
#include "uart4_dma.h"
#include "sbus.h"
#define DisArmed    0
#define Armed       1


#define  PITCH_CH    sbus_channel[1]
#define  ROLL_CH     sbus_channel[0]
#define  YAW_CH      sbus_channel[3]
#define  THR_CH      sbus_channel[2]


extern RemoterTypeDef  Remoter;
extern DroneStatusTypeDef DroneStatus;
extern CtrlerTypeDef Ctrler;

void remoter_task(void *pvParameters);
char CHECK_ARM_FLAG(void);
void Check_Fly_Mode(void);
void Check_Stick_Motion(void);

#endif
