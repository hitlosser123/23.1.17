#include "SensorsTask.h"
#include "Optical.h"
#include "us_100.h"
#include "tf_mini_plus.h"
#include "spl06.h"
#include "icm20602.h"
#include "ak8975.h"
#include "mag.h"
#include "RemoterTask.h"
#include "SINS.h"
#include "StabilizerTask.h"
#include "pid.h"

float groud_temp=25.0f,groud_press=101325.0f;		//
float pressure,temperature;
float baro_alt;	//unit m
float mag[3];	//磁力

void sensorsTask(void *pvParameters)
{
	EventBits_t uxBits;
	const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
	spl06_init();
	icm20602_init();
	ak8975_init();Mag_ST_Init();
	vTaskDelay(3000);	/*延时等待传感器稳定*/
	spl06_update();
	groud_press = spl06_get_pressure();
	groud_temp = spl06_get_temperature();
	DroneStatus.Is_GetingGyroZero=true;
	vTaskDelay(1000);
	get_gyro_zero();
	vTaskDelay(1000);
	DroneStatus.Is_GetingGyroZero=false;
//	USART1_TX_BUF[0]=0x55;USART1_DMA_TX_Enable(1);US_100_Update();
	while(1)
	{
		if(EventGroupHandler!=NULL)
		{
			uxBits = xEventGroupWaitBits(
								EventGroupHandler,	//事件标志组句柄
								get_gyro_BIT | get_angle_BIT | get_height_BIT | get_flow_BIT| got_SONAR_BIT ,
								pdTRUE,        /* BITs should be cleared before returning. */
								pdFALSE,       /* Don't wait for all bits, either bit will do. */
								xTicksToWait );/* Wait a maximum of 100ms for either bit to be set. */
			if( ( uxBits & ( get_gyro_BIT | get_angle_BIT ) ) == ( get_gyro_BIT | get_angle_BIT ) )
			{
//				MPU_Get_Gyroscope(&(sensors_gyrox),&(sensors_gyroy),&(sensors_gyroz));
//				MPU_Get_Accelerometer(&sensors_accx,&sensors_accy,&sensors_accz);
			}
			else	if( ( uxBits & get_gyro_BIT ) != 0 )
			{
//			MPU_Get_Gyroscope(&(sensors_gyrox),&(sensors_gyroy),&(sensors_gyroz));
//			MPU_Get_Accelerometer(&sensors_accx,&sensors_accy,&sensors_accz);
//			data.gyroxdata.stdunit=( sensors_gyrox - data.gyroxdata.zero)/MPU_Range;
//			data.gyroxdata.filtered=filterloop(data.gyroxdata.stdunit);
//			data.gyroydata.filtered = Butterworth50HzLPF(data.gyroydata.stdunit);
			}
			else if( ( uxBits & get_angle_BIT ) != 0 )
			{
			}
			else if( ( uxBits & get_flow_BIT ) != 0 )
			{
				//Optical_Update(  stSINS.Position[_Z]  );
			//	Optical_Update(  1.6);//Ctrler.Z_posPID.FB);//0.8  );
			}
			else if( ( uxBits & got_SONAR_BIT ) != 0 )
			{
	//			US_100_Update();
		//		TF_Mini_Plus_Update(USART1_RX_BUF);
			}
			else//超时
			{

//				READ_MPU9250_ACCEL();
//				READ_MPU9250_GYRO();
			}
		}
		else{vTaskDelay(10);} //事件标志组还没有，就延时10ms，也就是10个时钟节拍
	}
}
   

