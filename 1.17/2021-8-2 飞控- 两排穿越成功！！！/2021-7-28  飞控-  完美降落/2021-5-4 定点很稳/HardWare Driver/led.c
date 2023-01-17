#include "includes.h"
#include "led.h"
#include "x_gpio.h"
/********
D3
D2 
F4
E3
***********/

/**********
黄：  解锁   U0R   PA0
红：  小圈   U4R   PC4
绿：  顺时针 U0T   PA1
黄：  放在中间  U7R   PE0
*********/

void Led_Init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);//
	
	            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x04;
            HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xfb;
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_2);
	
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);
	
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_0,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
												 
//PA1
	MAP_GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_1,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
												 
												 
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//PC4
	MAP_GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
												 
												 
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//PE0
	MAP_GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

bool IsKeyPressed(void)//黄：  解锁   U0R   PA0
{
	if( GPIO_DATA(GPIO_PORTA_BASE,GPIO_PIN_0) )return false;
	else return true;
}

bool IsLittleCicle(void)// 红：  小圈   U4R   PC4
{
	if( GPIO_DATA(GPIO_PORTC_BASE,GPIO_PIN_4) )return false;
	else return true;	
}

bool IsClockWise(void)//绿：  顺时针 U0T   PA1
{
	if( GPIO_DATA(GPIO_PORTA_BASE,GPIO_PIN_1) )return false;
	else return true;	
}

bool IsTakeOffMiddle(void)//黄：  放在中间  U7R   PE0
{
	if( GPIO_DATA(GPIO_PORTE_BASE,GPIO_PIN_0) )return false;
	else return true;	
}

void Led_Test(void)
{
	Led_Init();
		while(1)
	{
		        SysCtlDelay(ui32SysClock / 10 / 3);// Delay for a bit.
        LED1_ON;LED2_OFF;LED3_OFF;LED4_OFF;
		
        SysCtlDelay(ui32SysClock / 10 / 3);// Delay for a bit.
		LED1_OFF;LED2_ON; LED3_OFF;LED4_OFF;
		
		SysCtlDelay(ui32SysClock / 10 / 3);// Delay for a bit.
		LED1_OFF;LED2_OFF;LED3_ON;LED4_OFF;
		
		SysCtlDelay(ui32SysClock / 10 / 3);// Delay for a bit.
		LED1_OFF;LED2_OFF;LED3_OFF;LED4_ON;
		

	}
}
