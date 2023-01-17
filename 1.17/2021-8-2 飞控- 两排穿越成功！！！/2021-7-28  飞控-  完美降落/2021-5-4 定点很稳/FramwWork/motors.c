#include "motors.h"
#include "RemoterTask.h"

MOTORTypeDef mymotor;
void Motors_Init(void)
{
	PWM_Output_Init();	
	Set_Zero_Motors();  
}

void Set_PWM_Motors(void)
{
	mymotor.motor1_pwm=mymotor.motor1-Motor_PWM_OFFSET;
	mymotor.motor2_pwm=mymotor.motor2-Motor_PWM_OFFSET;
	mymotor.motor3_pwm=mymotor.motor3-Motor_PWM_OFFSET;
	mymotor.motor4_pwm=mymotor.motor4-Motor_PWM_OFFSET;
	
	value_limit( mymotor.motor1_pwm , Motor_PWM_IDLE , Motor_PWM_MAX );		
	value_limit( mymotor.motor2_pwm , Motor_PWM_IDLE , Motor_PWM_MAX );	
	value_limit( mymotor.motor3_pwm , Motor_PWM_IDLE , Motor_PWM_MAX );		
	value_limit( mymotor.motor4_pwm , Motor_PWM_IDLE , Motor_PWM_MAX );	
	
	Set_4_Motors( mymotor.motor1_pwm, mymotor.motor2_pwm,\
								mymotor.motor3_pwm, mymotor.motor4_pwm );
}
 
void Set_Throttle_Motors(void)
{
	Set_4_Motors(Remoter.ThrCtrler,Remoter.ThrCtrler,Remoter.ThrCtrler,Remoter.ThrCtrler);
}

void Set_Zero_Motors(void)
{
	Set_4_Motors(Motor_PWM_MIN,Motor_PWM_MIN,Motor_PWM_MIN,Motor_PWM_MIN);		
}

void Set_IDLE_Motors(void)
{
	Set_4_Motors(Motor_PWM_IDLE,Motor_PWM_IDLE,Motor_PWM_IDLE,Motor_PWM_IDLE);			
}
