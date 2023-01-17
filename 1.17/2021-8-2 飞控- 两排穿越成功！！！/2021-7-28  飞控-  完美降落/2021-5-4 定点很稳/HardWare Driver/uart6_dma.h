#ifndef   __UART6_DMA_
#define   __UART6_DMA_
#include "includes.h"
#define DMA_MAX_LEN    1024//once 1024 datas
#define UART6_TXBUF_SIZE         30
#define UART6_RXBUF_SIZE         30//They do not need to be the same size.

#define      UDMA_CHANNEL_UART6RX         UDMA_CHANNEL_SSI0RX
#define      UDMA_CHANNEL_UART6TX         UDMA_CHANNEL_SSI0TX

extern  uint8_t UART6_TX_BUF[UART6_TXBUF_SIZE];
extern  uint8_t UART6_RX_BUFA[UART6_RXBUF_SIZE];
extern  uint8_t UART6_RX_BUFB[UART6_RXBUF_SIZE];

extern unsigned char UART6_RX_BUF[UART6_RXBUF_SIZE];
extern unsigned char Uart6_RX_POS;
void UART6_DMA_Init(void);

void USART6_DMA_TX_Enable(u16 length);
void Uart6_Tick_RX(void);

#endif

