#include "system_init.h"
#include "includes.h"

unsigned int ui32SysClock;// System clock rate in Hz.=120M

// The control table used by the uDMA controller.  This table must be aligned to a 1024 byte boundary.
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));

void System_Init(void)
{
//	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
//                                       SYSCTL_OSC_MAIN |
//                                       SYSCTL_USE_OSC), 25000000);
	
//	ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
//                                             SYSCTL_OSC_MAIN |
//                                             SYSCTL_USE_PLL |
//                                             SYSCTL_CFG_VCO_480), 120000000);
	
	    // Set the clocking to run at 50 MHz from the PLL.
//    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
//                       SYSCTL_OSC_MAIN);
	//����ϵͳʱ�� 80mhz
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);				
	
	ui32SysClock = ROM_SysCtlClockGet();
	
	  ROM_SysTickPeriodSet(ui32SysClock / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();
		
		ROM_FPULazyStackingEnable();
		
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	ROM_IntEnable(INT_UDMAERR);
	ROM_uDMAEnable();
	ROM_uDMAControlBaseSet(pui8ControlTable);//���Ʊ�
	
	
	
	  // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for SSI operation.
    InitConsole();
}

void InitConsole(void)
{
//! - UART0RX - PA0
//! - UART0TX - PA1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 16000000);
}


//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{     
	//while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
//    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
