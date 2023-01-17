#include "includes.h"
#include "drv_eeprom.h"

void EEPROM_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0)){}

	EEPROMInit();
}


uint32_t ui32EEPROMInit;
uint32_t pui32Data[2];
uint32_t pui32Read[2];
void  EEPROM_Test(void)
{

	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0)){}

	ui32EEPROMInit = EEPROMInit();

	if(ui32EEPROMInit != EEPROM_INIT_OK){  while(1); }

	pui32Data[0] = 0x12345678;
	pui32Data[1] = 0x56789abc;
	// Program some data into the EEPROM at address 0x400.
	EEPROMProgram(pui32Data, 0x400, sizeof(pui32Data));
	EEPROMRead(pui32Read, 0x400, sizeof(pui32Read));
}
