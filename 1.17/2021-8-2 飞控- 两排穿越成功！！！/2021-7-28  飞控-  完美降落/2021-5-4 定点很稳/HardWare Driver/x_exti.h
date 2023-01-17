#ifndef   __X_EXTI_
#define   __X_EXTI_
#include "includes.h"

#define   EXTI1_PORT     GPIO_PORTD_BASE
#define   EXTI1_PIN      GPIO_PIN_0
#define   EXTI1_INT_PIN  GPIO_INT_PIN_0
#define   EXTI1_CLK     SYSCTL_PERIPH_GPIOD

extern unsigned char flag;

void EXTI_Init(void);

#endif
