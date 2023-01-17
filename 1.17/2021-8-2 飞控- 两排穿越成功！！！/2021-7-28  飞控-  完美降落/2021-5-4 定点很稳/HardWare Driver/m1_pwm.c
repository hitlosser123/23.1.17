#include "system_init.h"
#include "m1_pwm.h"

        // This function provides a means of generating a constant length
        // delay.  The function delay (in cycles) = 3 * parameter.  Delay
        // 5 seconds arbitrarily.
        //SysCtlDelay((SysCtlClockGet() * 5) / 3);

#define PWM_Freq_HZ   400
void M1_PWM_Output_Init(void)
{
	uint16_t period;
	
  SysCtlPWMClockSet(SYSCTL_PWMDIV_8); // Set divider to sysclk/16
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); // Enable PWM peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIOB peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  // Use alternate function
  GPIOPinConfigure(GPIO_PA6_M1PWM2);

  // Use pin with PWM peripheral
  GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);//M1PWM2

  // Configure the PWM generator for count down mode with immediate updates to the parameters
  PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  
  PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  // The period is set to 2.5ms (400 Hz)
  period = SysCtlClockGet()/PWM_Freq_HZ/8; 
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, period); // Set the period
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, period);
  
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, period); // Set the period
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, period);
  // Start the timers in generator 0 and 1
  PWMGenEnable(PWM1_BASE, PWM_GEN_0);
  PWMGenEnable(PWM1_BASE, PWM_GEN_1);
  
  PWMGenEnable(PWM1_BASE, PWM_GEN_2);
  PWMGenEnable(PWM1_BASE, PWM_GEN_3);
  // Enable the outputs
  PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT , true);
  PWMPulseWidthSet(PWM1_BASE,PWM_OUT_2,0);//PA6
}
// 10000-20000
void Set_Heater(u16 pwmdata)
{
  PWMPulseWidthSet(PWM1_BASE,PWM_OUT_2,pwmdata);//PA6
}
