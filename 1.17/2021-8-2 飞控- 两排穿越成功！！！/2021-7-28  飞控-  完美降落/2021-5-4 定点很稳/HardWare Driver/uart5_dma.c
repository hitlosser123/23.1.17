#include "uart5_dma.h"

uint8_t UART5_TX_BUF[UART5_TXBUF_SIZE];
uint8_t UART5_RX_BUFA[UART5_RXBUF_SIZE];
uint8_t UART5_RX_BUFB[UART5_RXBUF_SIZE];

unsigned char UART5_RX_BUF[UART5_RXBUF_SIZE];
//  GPIO_PE4_U5RX   GPIO_PE5_U5TX
void UART5_DMA_Init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
	ROM_UARTConfigSetExpClk(UART5_BASE, ui32SysClock, 19200,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);//115200,8-n-1

		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    ROM_GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

														 
	ROM_UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

	ROM_UARTEnable(UART5_BASE);
//	ROM_UARTDMAEnable(UART5_BASE,UART_DMA_TX);// UART_DMA_RX | 

	
	
		
//	uDMAChannelAssign(UDMA_CH6_UART5RX);
//	uDMAChannelAssign(UDMA_CH7_UART5TX);
	
	
//	HWREG(UART5_BASE + UART_O_CTL) |= UART_CTL_LBE;//loopback mode

//	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART5RX,
//                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
//                                    UDMA_ATTR_HIGH_PRIORITY |
//                                    UDMA_ATTR_REQMASK);

//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART5RX | UDMA_PRI_SELECT,
//                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
//                              UDMA_ARB_4);

//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART5RX | UDMA_ALT_SELECT,
//                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
//                              UDMA_ARB_4);

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART5RX | UDMA_PRI_SELECT,
//                               UDMA_MODE_PINGPONG,
//                               (void *)(UART5_BASE + UART_O_DR),
//                               UART5_RX_BUFA, sizeof(UART5_RX_BUFA));

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART5RX | UDMA_ALT_SELECT,
//                               UDMA_MODE_PINGPONG,
//                               (void *)(UART5_BASE + UART_O_DR),
//                               UART5_RX_BUFB, sizeof(UART5_RX_BUFB));

//	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART5TX,
//                                    UDMA_ATTR_ALTSELECT |
//                                    UDMA_ATTR_HIGH_PRIORITY |
//                                    UDMA_ATTR_REQMASK);
// 
//	ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART5TX, UDMA_ATTR_USEBURST);
//															 
//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART5TX | UDMA_PRI_SELECT,
//                              UDMA_SIZE_8 | UDMA_SRC_INC_8 |
//                              UDMA_DST_INC_NONE |
//                              UDMA_ARB_4);

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART5TX | UDMA_PRI_SELECT,
//                               UDMA_MODE_BASIC, UART5_TX_BUF,
//                               (void *)(UART5_BASE + UART_O_DR),
//                               sizeof(UART5_TX_BUF));

//	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART5RX);
//	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART5TX);

//	ROM_UARTIntEnable(UART5_BASE, UART_INT_DMATX | UART_INT_DMARX);    // Enable the UART DMA TX/RX interrupts.
//	ROM_IntEnable(INT_UART5);
}

void UART5IntHandler(void)
{
    uint32_t ui32Status;
//    uint32_t ui32Mode;

    ui32Status = ROM_UARTIntStatus(UART5_BASE, 1);
    ROM_UARTIntClear(UART5_BASE, ui32Status);

//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART5RX | UDMA_PRI_SELECT);
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART5RX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_PINGPONG,
//                                   (void *)(UART5_BASE + UART_O_DR),
//                                   UART5_RX_BUFA, sizeof(UART5_RX_BUFA));
//    }

//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART5RX | UDMA_ALT_SELECT);
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART5RX | UDMA_ALT_SELECT,
//                                   UDMA_MODE_PINGPONG,
//                                   (void *)(UART5_BASE + UART_O_DR),
//                                   UART5_RX_BUFB, sizeof(UART5_RX_BUFB));
//    }

//    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART5TX))
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART5TX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_BASIC, UART5_TX_BUF,
//                                   (void *)(UART5_BASE + UART_O_DR),
//                                   sizeof(UART5_TX_BUF));
//        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART5TX);
//    }
}

void USART5_DMA_TX_Enable(u16 length)
{
    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART5TX))
    {
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART5TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, UART5_TX_BUF,
                                   (void *)(UART5_BASE + UART_O_DR),
                                   length );
        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART5TX);
    }
}

unsigned char Uart5_RX_POS=0;
void Uart5_Tick_RX(void)
{
	while(1)
	{
		if(UARTCharsAvail(UART5_BASE))
		{
			if(Uart5_RX_POS<UART5_RXBUF_SIZE)
					UART5_RX_BUF[Uart5_RX_POS++]=UARTCharGetNonBlocking(UART5_BASE);
		}
		else break;
	}
}
