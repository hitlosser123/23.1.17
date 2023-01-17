#ifndef  __X_GPIO_
#define  __X_GPIO_
#include "includes.h"


void X_GPIO_Output_Init(uint32_t ui32Port, uint8_t ui8Pins,uint32_t ui32SYSPerial);
void X_GPIO_Input_Init(uint32_t ui32Port, uint8_t ui8Pins,uint32_t ui32SYSPerial);

#define  X_GPIODirModeSet( ui32Port,  ui8Pins,  ui32PinIO)\
	HWREG(ui32Port+GPIO_O_DIR)=((ui32PinIO&1)?(HWREG(ui32Port+GPIO_O_DIR)|ui8Pins):(HWREG(ui32Port + GPIO_O_DIR) & ~(ui8Pins)));							
		
#define   GPIO_DATA( ui32Port,ui8Pins)			(HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))))		

#define  GPIO_DATA_All(ui32Port)     (HWREG(ui32Port + (GPIO_O_DATA + (0xFF << 2))))

#define  GPIO_H(ui32Port,ui8Pins)     HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))) = ui8Pins;
#define  GPIO_L(ui32Port,ui8Pins)     HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))) = 0;

extern unsigned  char inGPIOD;

#endif
