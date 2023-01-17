#ifndef   __UART5_DMA_
#define   __UART5_DMA_
#include "includes.h"
#define DMA_MAX_LEN    1024//once 1024 datas
#define UART5_TXBUF_SIZE         18
#define UART5_RXBUF_SIZE         20//They do not need to be the same size.

#define   UDMA_CHANNEL_UART5RX        UDMA_CHANNEL_ETH0RX
#define   UDMA_CHANNEL_UART5TX        UDMA_CHANNEL_ETH0TX

extern  uint8_t UART5_TX_BUF[UART5_TXBUF_SIZE];
extern  uint8_t UART5_RX_BUFA[UART5_RXBUF_SIZE];
extern  uint8_t UART5_RX_BUFB[UART5_RXBUF_SIZE];

extern unsigned char UART5_RX_BUF[UART5_RXBUF_SIZE];
extern unsigned char Uart5_RX_POS;

void UART5_DMA_Init(void);
void USART5_DMA_TX_Enable(u16 length);
void Uart5_Tick_RX(void);

#endif

