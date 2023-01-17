#include "x_exti.h"
unsigned char flag;
unsigned int count;

//  PJ0
void FaultISR()
{
	while(1)
	{
	}
}
void IntDefaultHandler()
{
	while(1)
	{
	}
}
void EXTI1IntHandler()
{
	 uint32_t ui32Status;
    ui32Status = GPIOIntStatus(EXTI1_PORT, true);
    GPIOIntClear(EXTI1_PORT, ui32Status);
//    if(flag==0)flag=1;else flag=0;
//	count++;
   // GPIOIntClear(EXTI2_PORT,EXTI2_INT_PIN);
//	OV7725_EXTI_CallBack();
}

//  LED   PF4 PF0
void EXTI_Init(void)
{
    SysCtlPeripheralEnable(EXTI1_CLK);

	  GPIOPinTypeGPIOInput(EXTI1_PORT, EXTI1_PIN);	
    GPIOIntTypeSet(EXTI1_PORT,EXTI1_PIN,GPIO_FALLING_EDGE);	//设置中断类型
    GPIOIntEnable(EXTI1_PORT,EXTI1_PIN);	    //使能中断
		GPIOIntRegister(EXTI1_PORT, EXTI1IntHandler);  //注册一个中断处理句柄		
}
