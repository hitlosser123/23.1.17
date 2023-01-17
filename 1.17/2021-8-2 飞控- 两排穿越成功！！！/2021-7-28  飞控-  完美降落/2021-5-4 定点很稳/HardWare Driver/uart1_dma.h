#ifndef   __UART_DMA_
#define   __UART_DMA_
#include "includes.h"
#define DMA_MAX_LEN    1024//once 1024 datas
#define UART1_TXBUF_SIZE         (50)
#define UART1_RXBUF_SIZE        50//They do not need to be the same size.

extern  uint8_t UART1_TX_BUF[UART1_TXBUF_SIZE];
extern  uint8_t UART1_RX_BUFA[UART1_RXBUF_SIZE];
extern  uint8_t UART1_RX_BUFB[UART1_RXBUF_SIZE];
extern  uint8_t USART1_RX_BUF[UART1_RXBUF_SIZE];



void UART1_DMA_Init(void);

extern unsigned char Uart1_RX_POS;
void Uart1_Tick_RX(void);

#endif

