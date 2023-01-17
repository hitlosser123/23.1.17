#include "uart3_dma.h"

uint8_t UART3_TX_BUF[UART3_TXBUF_SIZE];
uint8_t UART3_RX_BUFA[UART3_RXBUF_SIZE];
uint8_t UART3_RX_BUFB[UART3_RXBUF_SIZE];

uint8_t USART3_RX_BUF[UART3_RXBUF_SIZE];

#define     UDMA_CHANNEL_UART3RX         UDMA_CHANNEL_ADC2
#define     UDMA_CHANNEL_UART3TX         UDMA_CHANNEL_ADC3
//  GPIO_PC7_U3TX    GPIO_PC6_U3RX
void UART3_DMA_Init(void)
{	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	ROM_UARTConfigSetExpClk(UART3_BASE, ui32SysClock, 115200,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);//115200,8-n-1

		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC6_U3RX);
    GPIOPinConfigure(GPIO_PC7_U3TX);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

														 
	ROM_UARTFIFOLevelSet(UART3_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

	ROM_UARTEnable(UART3_BASE);
	ROM_UARTDMAEnable(UART3_BASE, UART_DMA_RX | UART_DMA_TX);

	
	uDMAChannelAssign(UDMA_CH16_UART3RX);
	uDMAChannelAssign(UDMA_CH17_UART3TX);
	
	
//	HWREG(UART3_BASE + UART_O_CTL) |= UART_CTL_LBE;//loopback mode

	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART3RX,
                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART3RX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                              UDMA_ARB_4);

	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART3RX | UDMA_ALT_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                              UDMA_ARB_4);

	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART3RX | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(UART3_BASE + UART_O_DR),
                               UART3_RX_BUFA, sizeof(UART3_RX_BUFA));

	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART3RX | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(UART3_BASE + UART_O_DR),
                               UART3_RX_BUFB, sizeof(UART3_RX_BUFB));

	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART3TX,
                                    UDMA_ATTR_ALTSELECT |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);
 
	ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART3TX, UDMA_ATTR_USEBURST);
															 
	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART3TX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_8 |
                              UDMA_DST_INC_NONE |
                              UDMA_ARB_4);

	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART3TX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC, UART3_TX_BUF,
                               (void *)(UART3_BASE + UART_O_DR),
                               sizeof(UART3_TX_BUF));

	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART3RX);
	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART3TX);

	ROM_UARTIntEnable(UART3_BASE, UART_INT_DMATX | UART_INT_DMARX);    // Enable the UART DMA TX/RX interrupts.
	ROM_IntEnable(INT_UART3);
}

void UART3IntHandler(void)
{
    uint32_t ui32Status;
    uint32_t ui32Mode;

    ui32Status = ROM_UARTIntStatus(UART3_BASE, 1);
    ROM_UARTIntClear(UART3_BASE, ui32Status);

    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART3RX | UDMA_PRI_SELECT);
    if(ui32Mode == UDMA_MODE_STOP)
    {
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART3RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(UART3_BASE + UART_O_DR),
                                   UART3_RX_BUFA, sizeof(UART3_RX_BUFA));
    }

    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART3RX | UDMA_ALT_SELECT);
    if(ui32Mode == UDMA_MODE_STOP)
    {
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART3RX | UDMA_ALT_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(UART3_BASE + UART_O_DR),
                                   UART3_RX_BUFB, sizeof(UART3_RX_BUFB));
    }

//    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART3TX))
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART3TX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_BASIC, UART3_TX_BUF,
//                                   (void *)(UART3_BASE + UART_O_DR),
//                                   sizeof(UART3_TX_BUF));
//        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART3TX);
//    }
}

void USART3_DMA_TX_Enable(u16 length)
{
    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART3TX))
    {
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART3TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, UART3_TX_BUF,
                                   (void *)(UART3_BASE + UART_O_DR),
                                   length );
        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART3TX);
    }
}
