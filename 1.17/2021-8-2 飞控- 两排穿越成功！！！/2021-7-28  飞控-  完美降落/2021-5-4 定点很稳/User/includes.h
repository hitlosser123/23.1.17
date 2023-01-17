#ifndef __INCLUDES_
#define __INCLUDES_


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/ssi.h"
#include "driverlib/eeprom.h"
#include "driverlib/pwm.h"
#include "utils/cpu_usage.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"


#include "FreeRTOS.h"					//FreeRTOS π”√		   
#include "task.h" 
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"

#include "system_init.h"

#include "uart.h"
#include "uartstdio.h"

#include "config&param.h"

extern unsigned int SysTickCnt;
u32 getSysTickCnt(void);

#endif
