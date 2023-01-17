#include "drv_adc.h"
//!!!!!!!!!!   startup.s

uint32_t pui32ADC0Value[8];
float adc_voltage_Volt;
unsigned int cnt_int=0;
void ADC0Sequence0Handler(void)
{
	unsigned char i;
	unsigned int sum;
	
	ADCIntClear(ADC0_BASE, 0); // Clear the ADC interrupt flag.
	ADCSequenceDataGet(ADC0_BASE, 0, pui32ADC0Value);// Read ADC Value.

	for(i = 0;i < 8;i ++)
	{
	 sum += pui32ADC0Value[i];
	}
	adc_voltage_Volt = sum *3.3f/4096;
	
	cnt_int++;
}

void ADC_Init(void)
{
	IntMasterEnable();
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//PE0->AIN3
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);

	ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_EIGHTH, 1);
	ADCPhaseDelaySet(ADC0_BASE,ADC_PHASE_337_5);
	
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_ALWAYS,0);//ADC_TRIGGER_PROCESSOR, 0);// Enable sample sequence 0

	// Configure step 0--7 on sequence 0.  
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH3 |ADC_CTL_SHOLD_256);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH3 |ADC_CTL_SHOLD_256);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH3 |ADC_CTL_SHOLD_256);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH3 |ADC_CTL_SHOLD_256);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH3 |ADC_CTL_SHOLD_256);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH3 |ADC_CTL_SHOLD_256);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH3 |ADC_CTL_SHOLD_256);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH3 |ADC_CTL_SHOLD_256 | ADC_CTL_IE | ADC_CTL_END);

	ADCIntEnable(ADC0_BASE, 0);
	IntEnable(INT_ADC0SS0);// enable ADC0 sequence 0 interrupt

	ADCSequenceEnable(ADC0_BASE, 0);// Since sample sequence 0 is now configured, it must be enabled.

	ADCIntClear(ADC0_BASE, 0);//make sure the interrupt flag is cleared before we sample.
}
