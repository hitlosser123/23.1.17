#include "uart2_dma.h"
#include "includes.h"
#include "sbus.h"

uint8_t UART2_TX_BUF[UART2_TXBUF_SIZE];
uint8_t UART2_RX_BUFA[UART2_RXBUF_SIZE];
uint8_t UART2_RX_BUFB[UART2_RXBUF_SIZE];

uint8_t USART2_RX_BUF[UART2_RXBUF_SIZE];

unsigned int SBUS_ValidCnt=0;
unsigned int USART2_REC_CNT;


//   GPIO_PD6_U2RX     GPIO_PD7_U2TX
void UART2_DMA_Init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	ROM_UARTConfigSetExpClk(UART2_BASE, ROM_SysCtlClockGet(), 100000,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_TWO |
                           UART_CONFIG_PAR_ONE| UART_CONFIG_PAR_EVEN);//115200,8-n-1

		
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK)=GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE+GPIO_O_CR)=0xff;
	
	GPIOPinConfigure(GPIO_PD6_U2RX);
//	GPIOPinConfigure(GPIO_PD7_U2TX);
	ROM_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 );//| GPIO_PIN_7);

														 
//	ROM_UARTFIFOLevelSet(UART2_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

	ROM_UARTEnable(UART2_BASE);
	ROM_UARTDMAEnable(UART2_BASE, UART_DMA_RX );//| UART_DMA_TX);

//	HWREG(UART2_BASE + UART_O_CTL) |= UART_CTL_LBE;//loopback mode

	
	uDMAChannelAssign(UDMA_CH0_UART2RX);
//	uDMAChannelAssign(UDMA_CH1_UART2TX);

	
	
	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART2RX,
                                    UDMA_ATTR_ALTSELECT | //UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART2RX, UDMA_ATTR_USEBURST);

	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART2RX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                              UDMA_ARB_4);

//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART2RX | UDMA_ALT_SELECT,
//                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
//                              UDMA_ARB_4);

	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART2RX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)(UART2_BASE + UART_O_DR),
                               UART2_RX_BUFA, sizeof(UART2_RX_BUFA));

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART2RX | UDMA_ALT_SELECT,
//                               UDMA_MODE_PINGPONG,
//                               (void *)(UART2_BASE + UART_O_DR),
//                               UART2_RX_BUFB, sizeof(UART2_RX_BUFB));

															 
															 
															 
//	ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART2TX,
//                                    UDMA_ATTR_ALTSELECT |
//                                    UDMA_ATTR_HIGH_PRIORITY |
//                                    UDMA_ATTR_REQMASK);
// 
//	ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART2TX, UDMA_ATTR_USEBURST);
//															 
//	ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART2TX | UDMA_PRI_SELECT,
//                              UDMA_SIZE_8 | UDMA_SRC_INC_8 |
//                              UDMA_DST_INC_NONE |
//                              UDMA_ARB_4);

//	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART2TX | UDMA_PRI_SELECT,
//                               UDMA_MODE_BASIC, UART2_TX_BUF,
//                               (void *)(UART2_BASE + UART_O_DR),
//                               sizeof(UART2_TX_BUF));

	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART2RX);
//	ROM_uDMAChannelEnable(UDMA_CHANNEL_UART2TX);

	ROM_UARTIntEnable(UART2_BASE, UART_INT_DMARX);   //UART_INT_DMATX |   // Enable the UART DMA TX/RX interrupts.
	ROM_IntEnable(INT_UART2);
}

void UART2IntHandler(void)
{
    uint32_t ui32Status;
    uint32_t ui32Mode;
	
BaseType_t Result,xHigherPriorityTaskWoken; 
	
    ui32Status = ROM_UARTIntStatus(UART2_BASE, 1);
    ROM_UARTIntClear(UART2_BASE, ui32Status);

    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART2RX | UDMA_PRI_SELECT);
    if(ui32Mode == UDMA_MODE_STOP)
    {
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART2RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(UART2_BASE + UART_O_DR),
                                   UART2_RX_BUFA, sizeof(UART2_RX_BUFA));
																	 
																	 
			ROM_uDMAChannelEnable(UDMA_CHANNEL_UART2RX);
    }

					Result=xEventGroupSetBitsFromISR(EventGroupHandler, got_SBUS_BIT ,&xHigherPriorityTaskWoken);
			if(Result!=pdFAIL){portYIELD_FROM_ISR(xHigherPriorityTaskWoken);}
		
			if(UART2_RX_BUFA[0]==0x0f)
			{
				SBUS_ValidCnt++;
				sbus_decode(UART2_RX_BUFA);
			}
			if(UART2_RX_BUFA[25]==0x0f)
			{
				SBUS_ValidCnt++;
				sbus_decode(&UART2_RX_BUFA[25]);
			}
//		ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART2RX | UDMA_ALT_SELECT);
//		if(ui32Mode == UDMA_MODE_STOP)
//		{
//				ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART2RX | UDMA_ALT_SELECT,
//																	 UDMA_MODE_PINGPONG,
//																	 (void *)(UART2_BASE + UART_O_DR),
//																	 UART2_RX_BUFB, sizeof(UART2_RX_BUFB));
//		}

//    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_USBEP1TX))
//    {
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_USBEP1TX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_BASIC, UART2_TX_BUF,
//                                   (void *)(UART2_BASE + UART_O_DR),
//                                   sizeof(UART2_TX_BUF));
//        ROM_uDMAChannelEnable(UDMA_CHANNEL_USBEP1TX);
//    }
}

void USART2_DMA_TX_Enable(u16 length)
{
	        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_USBEP1TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, UART2_TX_BUF,
                                   (void *)(UART2_BASE + UART_O_DR),
                                   length );
        ROM_uDMAChannelEnable(UDMA_CHANNEL_USBEP1TX);
}

