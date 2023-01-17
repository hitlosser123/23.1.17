#include "x_gpio.h"
unsigned  char inGPIOD;
//  GPIO_DIR_MODE_IN
//void   X_GPIODirModeSet(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32PinIO)
//	{   
//		HWREG(ui32Port+GPIO_O_DIR)=((ui32PinIO&1)?(HWREG(ui32Port+GPIO_O_DIR)|ui8Pins):(HWREG(ui32Port + GPIO_O_DIR) & ~(ui8Pins)));							
//		HWREG(ui32Port+GPIO_O_AFSEL)=((ui32PinIO&2)?(HWREG(ui32Port + GPIO_O_AFSEL)|ui8Pins):(HWREG(ui32Port + GPIO_O_AFSEL) &~(ui8Pins)));	
//	}

void X_GPIO_Output_Init(uint32_t ui32Port, uint8_t ui8Pins,uint32_t ui32SYSPerial)
{
	SysCtlPeripheralEnable(ui32SYSPerial);

	GPIOPadConfigSet(ui32Port, ui8Pins, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  GPIODirModeSet(ui32Port, ui8Pins, GPIO_DIR_MODE_OUT);
  GPIOPinWrite(ui32Port,ui8Pins,ui8Pins);
}

void X_GPIO_Input_Init(uint32_t ui32Port, uint8_t ui8Pins,uint32_t ui32SYSPerial)
{
	SysCtlPeripheralEnable(ui32SYSPerial);

	GPIOPadConfigSet(ui32Port, ui8Pins, GPIO_STRENGTH_10MA, GPIO_PIN_TYPE_STD);
  GPIODirModeSet(ui32Port, ui8Pins, GPIO_DIR_MODE_IN);
  GPIOPinWrite(ui32Port,ui8Pins,ui8Pins);
}
