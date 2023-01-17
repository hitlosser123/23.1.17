#ifndef   __SYSTEM_INIT_
#define   __SYSTEM_INIT_

extern unsigned int ui32SysClock;// System clock rate in Hz.=120M
extern unsigned char pui8ControlTable[1024] __attribute__ ((aligned(1024)));

#define SYSTICKS_PER_SECOND     configTICK_RATE_HZ


void System_Init(void);
void InitConsole(void);

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

#endif
