#include "uart1_dma.h"

uint8_t UART1_TX_BUF[UART1_TXBUF_SIZE];
uint8_t UART1_RX_BUFA[UART1_RXBUF_SIZE];
uint8_t UART1_RX_BUFB[UART1_RXBUF_SIZE];

uint8_t USART1_RX_BUF[UART1_RXBUF_SIZE];

void UART1_DMA_Init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1);

	ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 500000,
													UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
													UART_CONFIG_PAR_NONE);

	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);



	ROM_UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

	ROM_UARTEnable(UART1_BASE);
	ROM_UARTDMAEnable(UART1_BASE, UART_DMA_TX);//UART_DMA_RX | 

	ROM_IntEnable(INT_UART1);

//	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART1RX,
//																	UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
//																	UDMA_ATTR_HIGH_PRIORITY |
//																	UDMA_ATTR_REQMASK);

//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
//														UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
//														UDMA_ARB_4);

//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT,
//														UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
//														UDMA_ARB_4);

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
//														 UDMA_MODE_PINGPONG,
//														 (void *)(UART1_BASE + UART_O_DR),
//														 UART1_RX_BUFA, sizeof(UART1_RX_BUFA));

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT,
//														 UDMA_MODE_PINGPONG,
//														 (void *)(UART1_BASE + UART_O_DR),
//														 UART1_RX_BUFB, sizeof(UART1_RX_BUFB));

	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART1TX,
																	UDMA_ATTR_ALTSELECT |
																	UDMA_ATTR_HIGH_PRIORITY |
																	UDMA_ATTR_REQMASK);

	ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART1TX, UDMA_ATTR_USEBURST);

	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART1TX | UDMA_PRI_SELECT,
														UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
														UDMA_ARB_4);
														
	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART1TX | UDMA_PRI_SELECT,
														 UDMA_MODE_BASIC, UART1_TX_BUF,
														 (void *)(UART1_BASE + UART_O_DR),
														 sizeof(UART1_TX_BUF));

//	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART1RX);
	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART1TX);
}


void UART1IntHandler(void)
{
    uint32_t ui32Status;
//    uint32_t ui32Mode;

    ui32Status = ROM_UARTIntStatus(UART1_BASE, 1);
    ROM_UARTIntClear(UART1_BASE, ui32Status);

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

unsigned char Uart1_RX_POS=0;
void Uart1_Tick_RX(void)
{
	while(1)
	{
		if(UARTCharsAvail(UART1_BASE))
		{
			if(Uart1_RX_POS<UART1_RXBUF_SIZE)USART1_RX_BUF[Uart1_RX_POS++]=UARTCharGetNonBlocking(UART1_BASE);
		}
		else break;
	}
}
