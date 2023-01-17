#include "uart6_dma.h"

uint8_t UART6_TX_BUF[UART6_TXBUF_SIZE];
uint8_t UART6_RX_BUFA[UART6_RXBUF_SIZE];
uint8_t UART6_RX_BUFB[UART6_RXBUF_SIZE];

unsigned char UART6_RX_BUF[UART6_RXBUF_SIZE];

//!!!!!!!!!!!  launchpad  没有引出这两个脚，暂未测试！！！！！！

//   GPIO_PD4_U6RX    GPIO_PD5_U6TX
void UART6_DMA_Init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
	ROM_UARTConfigSetExpClk(UART6_BASE, ui32SysClock, 115200,//9600,sonar    115200tof
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);//115200,8-n-1

		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinConfigure(GPIO_PD4_U6RX);
    GPIOPinConfigure(GPIO_PD5_U6TX);
    ROM_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

														 
	ROM_UARTFIFOLevelSet(UART6_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

	ROM_UARTEnable(UART6_BASE);

	
	
	

	//UARTFIFOEnable(UART6_BASE);

	
//	ROM_UARTDMAEnable(UART6_BASE, UART_DMA_RX | UART_DMA_TX);

//	
//		
//	uDMAChannelAssign(UDMA_CH10_UART6RX);
//	uDMAChannelAssign(UDMA_CH11_UART6TX);	
//	
//	
////	HWREG(UART6_BASE + UART_O_CTL) |= UART_CTL_LBE;//loopback mode

//	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART6RX,
//                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
//                                    UDMA_ATTR_HIGH_PRIORITY |
//                                    UDMA_ATTR_REQMASK);

//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART6RX | UDMA_PRI_SELECT,
//                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
//                              UDMA_ARB_4);

//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART6RX | UDMA_ALT_SELECT,
//                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
//                              UDMA_ARB_4);

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART6RX | UDMA_PRI_SELECT,
//                               UDMA_MODE_PINGPONG,
//                               (void *)(UART6_BASE + UART_O_DR),
//                               UART6_RX_BUFA, sizeof(UART6_RX_BUFA));

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART6RX | UDMA_ALT_SELECT,
//                               UDMA_MODE_PINGPONG,
//                               (void *)(UART6_BASE + UART_O_DR),
//                               UART6_RX_BUFB, sizeof(UART6_RX_BUFB));

//	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART6TX,
//                                    UDMA_ATTR_ALTSELECT |
//                                    UDMA_ATTR_HIGH_PRIORITY |
//                                    UDMA_ATTR_REQMASK);
// 
//	ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART6TX, UDMA_ATTR_USEBURST);
//															 
//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART6TX | UDMA_PRI_SELECT,
//                              UDMA_SIZE_8 | UDMA_SRC_INC_8 |
//                              UDMA_DST_INC_NONE |
//                              UDMA_ARB_4);

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART6TX | UDMA_PRI_SELECT,
//                               UDMA_MODE_BASIC, UART6_TX_BUF,
//                               (void *)(UART6_BASE + UART_O_DR),
//                               sizeof(UART6_TX_BUF));

//	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART6RX);
//	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART6TX);

//	ROM_UARTIntEnable(UART6_BASE, UART_INT_DMATX | UART_INT_DMARX);    // Enable the UART DMA TX/RX interrupts.
//	ROM_IntEnable(INT_UART6);
}

void UART6IntHandler(void)
{
    uint32_t ui32Status;
//    uint32_t ui32Mode;

    ui32Status = ROM_UARTIntStatus(UART6_BASE, 1);
    ROM_UARTIntClear(UART6_BASE, ui32Status);

//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART6RX | UDMA_PRI_SELECT);
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART6RX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_PINGPONG,
//                                   (void *)(UART6_BASE + UART_O_DR),
//                                   UART6_RX_BUFA, sizeof(UART6_RX_BUFA));
//    }

//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART6RX | UDMA_ALT_SELECT);
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART6RX | UDMA_ALT_SELECT,
//                                   UDMA_MODE_PINGPONG,
//                                   (void *)(UART6_BASE + UART_O_DR),
//                                   UART6_RX_BUFB, sizeof(UART6_RX_BUFB));
//    }

//    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART6TX))
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART6TX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_BASIC, UART6_TX_BUF,
//                                   (void *)(UART6_BASE + UART_O_DR),
//                                   sizeof(UART6_TX_BUF));
//        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART6TX);
//    }
}

void USART6_DMA_TX_Enable(u16 length)
{
    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART6TX))
    {
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART6TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, UART6_TX_BUF,
                                   (void *)(UART6_BASE + UART_O_DR),
                                   length );
        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART6TX);
    }
}

unsigned char Uart6_RX_POS=0;
void Uart6_Tick_RX(void)
{
	while(1)
	{
		if(UARTCharsAvail(UART6_BASE))
		{
			if(Uart6_RX_POS<UART6_RXBUF_SIZE)
					UART6_RX_BUF[Uart6_RX_POS++]=UARTCharGetNonBlocking(UART6_BASE);
		}
		else break;
	}
}
