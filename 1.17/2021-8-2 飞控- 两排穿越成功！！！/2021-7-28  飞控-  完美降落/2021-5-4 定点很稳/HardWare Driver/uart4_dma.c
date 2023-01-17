#include "uart4_dma.h"

uint8_t UART4_TX_BUF[UART4_TXBUF_SIZE];
uint8_t UART4_RX_BUFA[UART4_RXBUF_SIZE];
uint8_t UART4_RX_BUFB[UART4_RXBUF_SIZE];

//#define  UDMA_CHANNEL_UART4RX        UDMA_CHANNEL_TMR0A
//#define  UDMA_CHANNEL_UART4TX        UDMA_CHANNEL_TMR0B
//  GPIO_PC4_U4RX   GPIO_PC5_U4TX
unsigned short Uart4_Rec_CNT;

void UART4_DMA_Init(void)
{

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
//	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART4);
	ROM_UARTConfigSetExpClk(UART4_BASE, ui32SysClock, 460800,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);//115200,8-n-1

		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC4_U4RX);
    GPIOPinConfigure(GPIO_PC5_U4TX);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

														 
	ROM_UARTFIFOLevelSet(UART4_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

	ROM_UARTEnable(UART4_BASE);
	
	
//	
	ROM_UARTDMAEnable(UART4_BASE, UART_DMA_RX | UART_DMA_TX);

	
	uDMAChannelAssign(UDMA_CH18_UART4RX);
	uDMAChannelAssign(UDMA_CH19_UART4TX);
	
	
//	HWREG(UART4_BASE + UART_O_CTL) |= UART_CTL_LBE;//loopback mode

	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART4RX,
                                  UDMA_ATTR_ALTSELECT | //  UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART4RX,UDMA_ATTR_USEBURST);

	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART4RX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                              UDMA_ARB_4);

//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART4RX | UDMA_ALT_SELECT,
//                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
//                              UDMA_ARB_4);

	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART4RX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)(UART4_BASE + UART_O_DR),
                               UART4_RX_BUFA, sizeof(UART4_RX_BUFA));

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART4RX | UDMA_ALT_SELECT,
//                               UDMA_MODE_PINGPONG,
//                               (void *)(UART4_BASE + UART_O_DR),
//                               UART4_RX_BUFB, sizeof(UART4_RX_BUFB));

	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART4TX,
                                    UDMA_ATTR_ALTSELECT |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);
 
	ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART4TX, UDMA_ATTR_USEBURST);
															 
	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART4TX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_8 |
                              UDMA_DST_INC_NONE |
                              UDMA_ARB_4);

	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART4TX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC, UART4_TX_BUF,
                               (void *)(UART4_BASE + UART_O_DR),
                               sizeof(UART4_TX_BUF));

	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART4RX);
	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART4TX);

	ROM_UARTIntEnable(UART4_BASE, UART_INT_DMATX );//UART_INT_DMATX | );    // Enable the UART DMA TX/RX interrupts.



	ROM_IntEnable(INT_UART4);
}
unsigned int Uart4_IRQ_CNT=0;
void UART4IntHandler(void)
{
    uint32_t ui32Status;
    uint32_t ui32Mode;
	

    ui32Status = ROM_UARTIntStatus(UART4_BASE, 1);
    ROM_UARTIntClear(UART4_BASE, ui32Status);

		Uart4_IRQ_CNT++;
	
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART4RX | UDMA_PRI_SELECT);
    if(ui32Mode == UDMA_MODE_STOP)
    {
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART4RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(UART4_BASE + UART_O_DR),
                                   UART4_RX_BUFA, sizeof(UART4_RX_BUFA));
	   ROM_uDMAChannelEnable(UDMA_CHANNEL_UART4RX);
    }
    
	

		
//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART4RX | UDMA_ALT_SELECT);
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART4RX | UDMA_ALT_SELECT,
//                                   UDMA_MODE_PINGPONG,
//                                   (void *)(UART4_BASE + UART_O_DR),
//                                   UART4_RX_BUFB, sizeof(UART4_RX_BUFB));
//    }

//    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART4TX))
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART4TX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_BASIC, UART4_TX_BUF,
//                                   (void *)(UART4_BASE + UART_O_DR),
//                                   sizeof(UART4_TX_BUF));
//        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART4TX);
//    }
}

void USART4_DMA_TX_Enable(u16 length)
{
    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_UART4TX))
    {
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART4TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, UART4_TX_BUF,
                                   (void *)(UART4_BASE + UART_O_DR),
                                   length );
        ROM_uDMAChannelEnable(UDMA_CHANNEL_UART4TX);
    }
}
uint8_t UART4_RX_BUF[UART4_RXBUF_SIZE];
uint32_t Uart4_RX_POS=0;

void Uart4_Tick_RX(void)
{
//	while(1)
//	{
//		if(UARTCharsAvail(UART4_BASE))
//		{
//			if(Uart4_RX_POS<UART4_RXBUF_SIZE)
//					UART4_RX_BUF[Uart4_RX_POS++]=UARTCharGetNonBlocking(UART4_BASE);
//		}
//		else break;
//	}
}
