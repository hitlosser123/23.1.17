#include "includes.h"
#include "spi_master.h"

void SPI0_Test(void)
{
    uint32_t pui32DataRx[3];


    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);

    SSIEnable(SSI0_BASE);

    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0])){ }

	while(1)
	{
		SysCtlDelay(100);SSIDataPut(SSI0_BASE, 'S');while(SSIBusy(SSI0_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI0_BASE, 'P');while(SSIBusy(SSI0_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI0_BASE, 'I');while(SSIBusy(SSI0_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI0_BASE, 'T');while(SSIBusy(SSI0_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI0_BASE, 'E');while(SSIBusy(SSI0_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI0_BASE, 'S');while(SSIBusy(SSI0_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI0_BASE, 'T');while(SSIBusy(SSI0_BASE)) ;//等待发送完毕
	}
     //   SSIDataGet(SSI0_BASE, &pui32DataRx[ui32Index]);
      //  pui32DataRx[ui32Index] &= 0x00FF;
}




void  SPI1_Test(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinConfigure(GPIO_PF2_SSI1CLK);
	GPIOPinConfigure(GPIO_PF3_SSI1FSS);
	GPIOPinConfigure(GPIO_PF1_SSI1TX);
	GPIOPinTypeSSI(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_2);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0 , SSI_MODE_MASTER, 1000000, 8);
	SSIEnable(SSI1_BASE);

	while(1)
	{
		SysCtlDelay(100);SSIDataPut(SSI1_BASE, 'S');while(SSIBusy(SSI1_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI1_BASE, 'P');while(SSIBusy(SSI1_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI1_BASE, 'I');while(SSIBusy(SSI1_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI1_BASE, 'T');while(SSIBusy(SSI1_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI1_BASE, 'E');while(SSIBusy(SSI1_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI1_BASE, 'S');while(SSIBusy(SSI1_BASE)) ;//等待发送完毕
		SysCtlDelay(100);SSIDataPut(SSI1_BASE, 'T');while(SSIBusy(SSI1_BASE)) ;//等待发送完毕
	}
}
