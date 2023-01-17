#ifndef __MOTORS_H
#define __MOTORS_H	 
#include "config&param.h"
#include "m0_pwm.h"

#define M1  TIM4->CCR2
#define M2  TIM4->CCR3
#define M3  TIM4->CCR4
#define M4  TIM4->CCR1


	
	
	
#ifndef _BRUSHLESS
#define _BRUSHLESS
#endif
  
//#ifndef F330
//#define  F330
//#endif
//#ifndef  X215
//#define X215
//#endif
#ifndef  F380
#define F380
#endif

#ifndef  Propeller9450
#define  Propeller9450
#endif

//#ifndef  Propeller9443
//#define  Propeller9443
//#endif

#ifdef   _BRUSHLESS//无刷油门对应PWM值
#define Motor_PWM_MAX 4000
#define Motor_PWM_MIN 2000
#define Motor_PWM_IDLE   2280//怠速
#define Motor_PWM_OFFSET 0
	#ifdef  F330
	#define Throttle_threshold  3250//F330 3070   .//F330+脚架 3370
	#endif
	#ifdef   X215
	#define Throttle_threshold  3000
	#endif
	#ifdef   F380
		#ifdef Propeller9450
		#define Throttle_threshold  3350
		#endif
		#ifdef Propeller9443
		#define Throttle_threshold  3530
		#endif
	#endif	
#define Target_height   1.0
#define PWM_Period 5000-1//无刷油门对应PWM值
#define PWM_Prescaler    36-1
#endif

#ifdef   _BRUSH//有刷油门对应PWM值
#define Motor_PWM_MAX 2000
#define Motor_PWM_IDLE   200//怠速
#define Motor_PWM_MIN 0
#define Motor_PWM_OFFSET 2000
#define Throttle_threshold  2750
#define Target_height   0.5
#define PWM_Period 2000-1//有刷油门对应PWM值
#define PWM_Prescaler    36-1
#endif

extern MOTORTypeDef mymotor;

void Set_PWM_Motors(void);
void Set_Zero_Motors(void);
void Motors_Init(void);
void Set_Throttle_Motors(void);
void Set_IDLE_Motors(void);

#endif
