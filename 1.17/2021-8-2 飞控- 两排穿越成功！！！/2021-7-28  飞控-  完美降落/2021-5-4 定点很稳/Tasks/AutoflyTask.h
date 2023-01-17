#ifndef   __AUTOFLY_TASK_H_
#define   __AUTOFLY_TASK_H_
#include "includes.h"

extern unsigned char KeySDKflag,SDK_StateChangeFlag,SDK_DelayWakeFlag,ShotQRCodeFlag,SDKLandFlag;

extern float x_test ;
extern float y_test ;

void AutoflyTask(void *pvParameters);


void SDK_StateMachine_Reset(void);
void SDK_StateMachine_Loop(void);
void SDK_StateMachine_Init(void);

void SDK_Set_V_Loc(void);
void SDK_Set_H_Loc(void);
void SDK_Set_Gyroz(void);
void SDK_Set_Pos_Loc(void);
void SDK_Set_Yaw(void);
float SDK_yaw_gan_cnt(u8 yawcnt);

#endif
