#include "system_init.h"
#include "m0_pwm.h"

        // This function provides a means of generating a constant length
        // delay.  The function delay (in cycles) = 3 * parameter.  Delay
        // 5 seconds arbitrarily.
        //SysCtlDelay((SysCtlClockGet() * 5) / 3);

#define PWM_Freq_HZ   400
void PWM_Output_Init(void)
{
	uint16_t period;
	
  SysCtlPWMClockSet(SYSCTL_PWMDIV_8); // Set divider to sysclk/16
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  // Use alternate function
  GPIOPinConfigure(GPIO_PB6_M0PWM0);
  GPIOPinConfigure(GPIO_PB7_M0PWM1);
  GPIOPinConfigure(GPIO_PB4_M0PWM2);
  GPIOPinConfigure(GPIO_PB5_M0PWM3);

  // Use pin with PWM peripheral
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);//M0PWM0
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//M0PWM1
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);//M0PWM2
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);//M0PWM3

  // Configure the PWM generator for count down mode with immediate updates to the parameters
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  
  PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  // The period is set to 2.5ms (400 Hz)
  period = SysCtlClockGet()/PWM_Freq_HZ/8; 
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);
  
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, period); // Set the period
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
  // Start the timers in generator 0 and 1
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  
  PWMGenEnable(PWM0_BASE, PWM_GEN_2);
  PWMGenEnable(PWM0_BASE, PWM_GEN_3);
  // Enable the outputs
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,10000);//PB7
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,10000);//PB6
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,10000);//PB4
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,10000);//PB5
}
// 10000-20000
void Set_4_Motors(u16 m1,u16 m2,u16 m3,u16 m4)
{
	u16 m1x,m2x,m3x,m4x;
	m1x=m1*5;
	m2x=m2*5;
	m3x=m3*5;
	m4x=m4*5;
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,m1x);//PB7
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,m2x);//PB6
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,m3x);//PB4
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,m4x);//PB5
}
