#ifndef   __UART4_DMA_
#define   __UART4_DMA_
#include "includes.h"
#define DMA_MAX_LEN    1024//once 1024 datas
#define UART4_TXBUF_SIZE      18
#define UART4_RXBUF_SIZE         50//They do not need to be the same size.

#define  UDMA_CHANNEL_UART4RX        UDMA_CHANNEL_TMR0A
#define  UDMA_CHANNEL_UART4TX        UDMA_CHANNEL_TMR0B

extern  uint8_t UART4_TX_BUF[UART4_TXBUF_SIZE];
extern  uint8_t UART4_RX_BUFA[UART4_RXBUF_SIZE];
extern  uint8_t UART4_RX_BUFB[UART4_RXBUF_SIZE];

extern unsigned short Uart4_Rec_CNT;
extern unsigned char UART4_RX_BUF[UART4_RXBUF_SIZE];
extern uint32_t Uart4_RX_POS;
void UART4_DMA_Init(void);

void USART4_DMA_TX_Enable(u16 length);
void Uart4_Tick_RX(void);
#endif

