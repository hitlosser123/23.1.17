#ifndef   __UART7_DMA_
#define   __UART7_DMA_
#include "includes.h"
#define DMA_MAX_LEN    1024//once 1024 datas
#define UART7_TXBUF_SIZE         50
#define UART7_RXBUF_SIZE         100//They do not need to be the same size.

#define      UDMA_CHANNEL_UART7RX     UDMA_CHANNEL_TMR1A
#define      UDMA_CHANNEL_UART7TX     UDMA_CHANNEL_TMR1B

extern  uint8_t UART7_TX_BUF[UART7_TXBUF_SIZE];
extern  uint8_t UART7_RX_BUFA[UART7_RXBUF_SIZE];
extern  uint8_t UART7_RX_BUFB[UART7_RXBUF_SIZE];

extern uint8_t UART7_RX_BUF[UART7_RXBUF_SIZE];
extern unsigned short Uart7_Rec_CNT;
void UART7_DMA_Init(void);

void USART7_DMA_TX_Enable(u16 length);

#endif

