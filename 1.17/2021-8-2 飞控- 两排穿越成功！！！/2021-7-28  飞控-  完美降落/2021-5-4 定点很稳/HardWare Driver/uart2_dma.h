#ifndef   __UART2_DMA_
#define   __UART2_DMA_
#include "includes.h"
#define DMA_MAX_LEN    1024//once 1024 datas
#define UART2_TXBUF_SIZE         10
#define UART2_RXBUF_SIZE         50//They do not need to be the same size.

#define  UDMA_CHANNEL_UART2TX   UDMA_CHANNEL_USBEP1TX
#define  UDMA_CHANNEL_UART2RX   UDMA_CHANNEL_USBEP1RX

extern  uint8_t UART2_TX_BUF[UART2_TXBUF_SIZE];
extern  uint8_t UART2_RX_BUFA[UART2_RXBUF_SIZE];
extern  uint8_t UART2_RX_BUFB[UART2_RXBUF_SIZE];

extern  uint8_t USART2_RX_BUF[UART2_RXBUF_SIZE];

extern  unsigned int USART2_REC_CNT;

extern unsigned int SBUS_ValidCnt;

void UART2_DMA_Init(void);
void USART2_DMA_TX_Enable(u16 length);


#endif

