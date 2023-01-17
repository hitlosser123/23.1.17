#include "includes.h"
#include "uart1_dma.h"

void Timer0BIntHandler(void)
{
	    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);// Clear the timer interrupt flag.

}

void  Timer0AIntHandler(void)// The interrupt handler for the Timer0B interrupt.
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);// Clear the timer interrupt flag.

//        IntDisable(INT_TIMER0B);// Disable the Timer0B interrupt.
//        TimerIntDisable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);// Turn off Timer0B interrupt.
//        TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);// Clear any pending interrupt flag.

}
extern void xPortSysTickHandler(void);
unsigned int SysTickCnt=0;
void  SysTickHandler(void)
{
	SysTickCnt++;
	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
	{
			xPortSysTickHandler();	
	}
}

u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
		return xTaskGetTickCount();
	else
		return SysTickCnt;
}

void uDMAErrorHandler(void)
{
    uint32_t ui32Status;
    ui32Status = ROM_uDMAErrorStatusGet();
    if(ui32Status)
    {
        ROM_uDMAErrorStatusClear();
        //g_ui32uDMAErrCount++;
    }
}

void uDMAIntHandler(void)//memory channel
{
//    uint32_t ui32Mode;
//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_SW);
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//        //g_ui32MemXferCount++;
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SW, UDMA_MODE_AUTO,
//                                     g_ui32SrcBuf, g_ui32DstBuf,
//                                     MEM_BUFFER_SIZE);
//        ROM_uDMAChannelEnable(UDMA_CHANNEL_SW);
//        ROM_uDMAChannelRequest(UDMA_CHANNEL_SW);
//    }
//    else
//    {
//        g_ui32BadISR++;
//    }
}


void UART0IntHandler(void)
{
//    uint32_t ui32Status;
//    uint32_t ui32Mode;

//    ui32Status = ROM_UARTIntStatus(UART1_BASE, 1);
//    ROM_UARTIntClear(UART1_BASE, ui32Status);

//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT);
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_PINGPONG,
//                                   (void *)(UART1_BASE + UART_O_DR),
//                                   UART1_RX_BUFA, sizeof(UART1_RX_BUFA));
//    }

//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT);
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT,
//                                   UDMA_MODE_PINGPONG,
//                                   (void *)(UART1_BASE + UART_O_DR),
//                                   UART1_RX_BUFB, sizeof(UART1_RX_BUFB));
//    }

//    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART1TX))
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART1TX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_BASIC, UART1_TX_BUF,
//                                   (void *)(UART1_BASE + UART_O_DR),
//                                   sizeof(UART1_TX_BUF));
//        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART1TX);
//    }
}
