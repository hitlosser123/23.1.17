#include "uart7_dma.h"

uint8_t UART7_TX_BUF[UART7_TXBUF_SIZE];
uint8_t UART7_RX_BUFA[UART7_RXBUF_SIZE];
uint8_t UART7_RX_BUFB[UART7_RXBUF_SIZE];

uint8_t UART7_RX_BUF[UART7_RXBUF_SIZE];

unsigned short Uart7_Rec_CNT;

//   GPIO_PE0_U7RX    GPIO_PE1_U7TX
void UART7_DMA_Init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	ROM_UARTConfigSetExpClk(UART7_BASE, ui32SysClock, 9600,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);//115200,8-n-1

		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinConfigure(GPIO_PE0_U7RX);
    GPIOPinConfigure(GPIO_PE1_U7TX);
    ROM_GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

														 
	ROM_UARTFIFOLevelSet(UART7_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

	ROM_UARTEnable(UART7_BASE);
	ROM_UARTDMAEnable(UART7_BASE, UART_DMA_RX | UART_DMA_TX);

	
			
	uDMAChannelAssign(UDMA_CH20_UART7RX);
	uDMAChannelAssign(UDMA_CH21_UART7TX);	
	
	
//	HWREG(UART7_BASE + UART_O_CTL) |= UART_CTL_LBE;//loopback mode

	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART7RX,
                                    UDMA_ATTR_ALTSELECT |// UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART7RX,UDMA_ATTR_USEBURST);

	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART7RX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                              UDMA_ARB_4);

//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART7RX | UDMA_ALT_SELECT,
//                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
//                              UDMA_ARB_4);

	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART7RX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)(UART7_BASE + UART_O_DR),
                               UART7_RX_BUFA, sizeof(UART7_RX_BUFA));

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART7RX | UDMA_ALT_SELECT,
//                               UDMA_MODE_PINGPONG,
//                               (void *)(UART7_BASE + UART_O_DR),
//                               UART7_RX_BUFB, sizeof(UART7_RX_BUFB));

	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART7TX,
                                    UDMA_ATTR_ALTSELECT |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);
 
	ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART7TX, UDMA_ATTR_USEBURST);
															 
	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART7TX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_8 |
                              UDMA_DST_INC_NONE |
                              UDMA_ARB_4);

	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART7TX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC, UART7_TX_BUF,
                               (void *)(UART7_BASE + UART_O_DR),
                               sizeof(UART7_TX_BUF));

	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART7RX);
	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART7TX);

	ROM_UARTIntEnable(UART7_BASE, UART_INT_DMARX);//UART_INT_DMATX | );    // Enable the UART DMA TX/RX interrupts.
	ROM_IntEnable(INT_UART7);
}

unsigned int Uart7_IRQ_CNT=0;
void UART7IntHandler(void)
{
    uint32_t ui32Status;
    uint32_t ui32Mode;

    ui32Status = ROM_UARTIntStatus(UART7_BASE, 1);
    ROM_UARTIntClear(UART7_BASE, ui32Status);

	Uart7_IRQ_CNT++;
	
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART7RX | UDMA_PRI_SELECT);
    if(ui32Mode == UDMA_MODE_STOP)
    {
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART7RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(UART7_BASE + UART_O_DR),
                                   UART7_RX_BUFA, sizeof(UART7_RX_BUFA));
																	 
				ROM_uDMAChannelEnable(UDMA_CHANNEL_UART7RX);
    }

//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART7RX | UDMA_ALT_SELECT);
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART7RX | UDMA_ALT_SELECT,
//                                   UDMA_MODE_PINGPONG,
//                                   (void *)(UART7_BASE + UART_O_DR),
//                                   UART7_RX_BUFB, sizeof(UART7_RX_BUFB));
//    }

//    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART7TX))
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART7TX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_BASIC, UART7_TX_BUF,
//                                   (void *)(UART7_BASE + UART_O_DR),
//                                   sizeof(UART7_TX_BUF));
//        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART7TX);
//    }
}

void USART7_DMA_TX_Enable(u16 length)
{
    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART7TX))
    {
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART7TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, UART7_TX_BUF,
                                   (void *)(UART7_BASE + UART_O_DR),
                                   length );
        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART7TX);
    }
}
