#ifndef   __LED_
#define   __LED_

#include "includes.h"

#define  LED1_ON   GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);// Turn on the LED.
#define  LED1_OFF  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);// Turn off the LED.

#define  LED2_ON   GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);// Turn on the LED.
#define  LED2_OFF  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);// Turn off the LED.

#define  LED3_ON   GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);// Turn on the LED.
#define  LED3_OFF  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);// Turn off the LED.

#define  LED4_ON   GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_2, 0);// Turn on the LED.
#define  LED4_OFF  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_2, GPIO_PIN_2);// Turn off the LED.


#define  BEEP_ON 		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
#define  BEEP_OFF   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
		
		
#define LED_Yellow_ON    LED4_ON
#define LED_Green_ON     LED2_ON
#define LED_Blue_ON      LED1_ON
#define LED_Red_ON       LED3_ON

#define LED_Yellow_OFF    LED4_OFF
#define LED_Green_OFF     LED2_OFF
#define LED_Blue_OFF      LED1_OFF
#define LED_Red_OFF       LED3_OFF


void Led_Init(void);
void Led_Test(void);
bool IsKeyPressed(void);
bool IsLittleCicle(void);
bool IsClockWise(void);
bool IsTakeOffMiddle(void);

#endif

