#include "LedTask.h"
#include "RemoterTask.h"
#include "AutoflyTask.h"

#define abs(x) ( (x)>0?(x):-(x) )
void LED_task(void *pvParameters)
{
	LED_Blue_ON;

	while(1)
	{
		vTaskDelay(LED_DT ); 	

    if(DroneStatus.ARM_Status==DisArmed)
		{
		if(((abs(Ctrler.rollPID.FB))>3||abs(Ctrler.pitchPID.FB)>3))
		{
	  	LED_Red_ON;LED_Blue_OFF;LED_Yellow_OFF;LED_Green_OFF; //∫Ïµ∆¡¡
		}
		else
		{
		  LED_Red_OFF;LED_Blue_ON;LED_Yellow_OFF;LED_Green_OFF
		}
	  }
		
		else if(DroneStatus.ARM_Status ==Armed)
		{
			LED_Yellow_OFF;LED_Red_OFF;LED_Blue_OFF;LED_Green_ON;
		}
	}
}

