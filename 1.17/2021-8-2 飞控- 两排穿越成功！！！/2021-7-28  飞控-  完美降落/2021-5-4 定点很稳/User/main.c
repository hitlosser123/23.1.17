#include "includes.h"
#include "system_init.h"
#include "led.h"
#include "drv_eeprom.h"
#include "ak8975.h"
#include "spi1.h"
#include "m0_pwm.h"
#include "delay.h"
#include "icm20602.h"
#include "spl06.h"
#include "uart1_dma.h"
#include "uart2_dma.h"
#include "uart3_dma.h"
#include "uart4_dma.h"
#include "uart5_dma.h"
#include "uart6_dma.h"
#include "uart7_dma.h"
#include "x_gpio.h"
#include "AutoflyTask.h"
#include "DebugerTask.h"
#include "LedTask.h"
#include "RemoterTask.h"
#include "SensorsTask.h"
#include "StabilizerTask.h"
#include "motors.h"
#include "adrc.h"
#include "m1_pwm.h"

TaskHandle_t StartTask_Handler;//任务句柄
EventGroupHandle_t EventGroupHandler;	//事件标志组句柄
void start_task(void *pvParameters);//任务函数

int main(void)//x:roll   右为正    右加左减//1:右上   2：左下    3:左上    4：右下
{
	System_Init();
	//SPI0_Test();
	//SPI1_Test();
	Led_Init();//Led_Test();
	Motors_Init();
	EEPROM_Init();
	SPI1_Init();
	StructureInit();//初始化PID用到EEPROM
	
	UART1_DMA_Init();
	UART2_DMA_Init();
	UART3_DMA_Init();
	UART4_DMA_Init();
	UART5_DMA_Init();
	UART6_DMA_Init();
	UART7_DMA_Init();
	ADRC_Init(&ADRC_Pitch_Controller,&ADRC_Roll_Controller);
	
//	delay_ms(10000);//需要等待上电陀螺仪初始化
	
	//M1_PWM_Output_Init(); 
	
/*********
串口分配：
UART0:     console
UART1:	 OPTICAL
UART2:   RX-SBUS
UART3:   NC
UART4:   BT
UART5:   VISION
UART6:   SONAR/TOF
UART7:
	**************/
	

	
//		IMU_Temp=icm20602_get_temp();
//		if(IMU_Temp<=45.0f)
//		{
//			GPIO_H(ICM_Temp_Ctrl_PORT,ICM_Temp_Ctrl_IO);
//		}
//		else
//		{
//			GPIO_L(ICM_Temp_Ctrl_PORT,ICM_Temp_Ctrl_IO);
//		}


	

	
	xTaskCreate(start_task,"start_task",128,NULL, 1,&StartTask_Handler);
	
	vTaskStartScheduler(); 
  while(1){}
}//uxTaskGetStackHighWaterMark      vTaskList//  EEPROM    验证数据有效性    飞行模式

BaseType_t xReturn;
void start_task(void *pvParameters)//开始任务任务函数
{//一共可以分配的内存大概在4000-4500，需要的多了会任务创建失败，给的少了堆栈溢出hardfault
  taskENTER_CRITICAL();           //进入临界区
	EventGroupHandler=xEventGroupCreate();	 //创建事件标志组
	 
	xReturn=xTaskCreate( LED_task, "LED",150,NULL,1,NULL); printf("%ld\n",xReturn); 
	xReturn=xTaskCreate( sensorsTask, "SENSORS",500,NULL,sensorsTask_Prio,NULL);//printf("%ld\n",xReturn);
  xTaskCreate( stabilizerTask, "STABILIZER",500,NULL,stabilizerTask_Prio,NULL);printf("%ld\n",xReturn);
	xReturn=xTaskCreate( DebugerTask, "DEBUGER",500,NULL,datalinkTask_Prio,NULL); printf("%ld\n",xReturn);
	xReturn=xTaskCreate( remoter_task, "REMOTER",500,NULL,remoter_task_Prio,NULL); printf("%ld\n",xReturn);
	xReturn=xTaskCreate( UartRxDecodeTask, "UARTDECODE",500,NULL,remoter_task_Prio,NULL); printf("%ld\n",xReturn);
	xReturn=xTaskCreate( AutoflyTask, "AUTOFLY",500,NULL,autofly_task_Prio,NULL); printf("%ld\n",xReturn);
//	
  vTaskDelete(StartTask_Handler); //删除开始任务
  taskEXIT_CRITICAL();            //退出临界区
}

#ifdef DEBUG// The error routine that is called if the driver library encounters an error.
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// This hook is called by FreeRTOS when an stack overflow error is detected.
void  vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    while(1){}
}
