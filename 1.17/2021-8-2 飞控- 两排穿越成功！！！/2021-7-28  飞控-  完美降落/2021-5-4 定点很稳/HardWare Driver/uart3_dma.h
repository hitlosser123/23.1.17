#ifndef   __UART3_DMA_
#define   __UART3_DMA_
#include "includes.h"
#define DMA_MAX_LEN    1024//once 1024 datas
#define UART3_TXBUF_SIZE         10
#define UART3_RXBUF_SIZE         10//They do not need to be the same size.

extern  uint8_t UART3_TX_BUF[UART3_TXBUF_SIZE];
extern  uint8_t UART3_RX_BUFA[UART3_RXBUF_SIZE];
extern  uint8_t UART3_RX_BUFB[UART3_RXBUF_SIZE];

extern  uint8_t USART3_RX_BUF[UART3_RXBUF_SIZE];

void UART3_DMA_Init(void);

void USART3_DMA_TX_Enable(u16 length);

#endif

