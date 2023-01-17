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

TaskHandle_t StartTask_Handler;//������
EventGroupHandle_t EventGroupHandler;	//�¼���־����
void start_task(void *pvParameters);//������

int main(void)//x:roll   ��Ϊ��    �Ҽ����//1:����   2������    3:����    4������
{
	System_Init();
	//SPI0_Test();
	//SPI1_Test();
	Led_Init();//Led_Test();
	Motors_Init();
	EEPROM_Init();
	SPI1_Init();
	StructureInit();//��ʼ��PID�õ�EEPROM
	
	UART1_DMA_Init();
	UART2_DMA_Init();
	UART3_DMA_Init();
	UART4_DMA_Init();
	UART5_DMA_Init();
	UART6_DMA_Init();
	UART7_DMA_Init();
	ADRC_Init(&ADRC_Pitch_Controller,&ADRC_Roll_Controller);
	
//	delay_ms(10000);//��Ҫ�ȴ��ϵ������ǳ�ʼ��
	
	//M1_PWM_Output_Init(); 
	
/*********
���ڷ��䣺
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
}//uxTaskGetStackHighWaterMark      vTaskList//  EEPROM    ��֤������Ч��    ����ģʽ

BaseType_t xReturn;
void start_task(void *pvParameters)//��ʼ����������
{//һ�����Է�����ڴ�����4000-4500����Ҫ�Ķ��˻����񴴽�ʧ�ܣ��������˶�ջ���hardfault
  taskENTER_CRITICAL();           //�����ٽ���
	EventGroupHandler=xEventGroupCreate();	 //�����¼���־��
	 
	xReturn=xTaskCreate( LED_task, "LED",150,NULL,1,NULL); printf("%ld\n",xReturn); 
	xReturn=xTaskCreate( sensorsTask, "SENSORS",500,NULL,sensorsTask_Prio,NULL);//printf("%ld\n",xReturn);
  xTaskCreate( stabilizerTask, "STABILIZER",500,NULL,stabilizerTask_Prio,NULL);printf("%ld\n",xReturn);
	xReturn=xTaskCreate( DebugerTask, "DEBUGER",500,NULL,datalinkTask_Prio,NULL); printf("%ld\n",xReturn);
	xReturn=xTaskCreate( remoter_task, "REMOTER",500,NULL,remoter_task_Prio,NULL); printf("%ld\n",xReturn);
	xReturn=xTaskCreate( UartRxDecodeTask, "UARTDECODE",500,NULL,remoter_task_Prio,NULL); printf("%ld\n",xReturn);
	xReturn=xTaskCreate( AutoflyTask, "AUTOFLY",500,NULL,autofly_task_Prio,NULL); printf("%ld\n",xReturn);
//	
  vTaskDelete(StartTask_Handler); //ɾ����ʼ����
  taskEXIT_CRITICAL();            //�˳��ٽ���
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
