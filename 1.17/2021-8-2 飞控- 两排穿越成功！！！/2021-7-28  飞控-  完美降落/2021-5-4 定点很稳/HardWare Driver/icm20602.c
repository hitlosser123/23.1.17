#include "icm20602.h"
#include "filter.h"
#include "spi1.h"
#include "StabilizerTask.h"
#include "x_gpio.h"
#include "delay.h"

static 	float _accel_scale;
static	float _gyro_scale;

#define GRAVITY_MSS 9.80665f
#define _ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)			//量程8G
#define _DEG_TO_RAD    0.0174532f 							//度转弧度
#define _RAD_TO_DEG    57.296f
//		icm20602_get_accel(acc);		
//		icm20602_get_gyro(gyro);

void ICM_20602_Update(void)
{
	//读6轴原始数据
	icm20602_get_gyro_adc(&data.ICM20602Data.GyroX.raw ,&data.ICM20602Data.GyroY.raw ,\
																&data.ICM20602Data.GyroZ.raw);
	
	icm20602_get_accel_adc(&data.ICM20602Data.AccX.raw ,&data.ICM20602Data.AccY.raw ,\
																&data.ICM20602Data.AccZ.raw);
	
	//校准
	data.ICM20602Data.GyroX.adjusted = data.ICM20602Data.GyroX.raw - data.ICM20602Data.GyroX.zero;
	data.ICM20602Data.GyroY.adjusted = data.ICM20602Data.GyroY.raw - data.ICM20602Data.GyroY.zero;
	data.ICM20602Data.GyroZ.adjusted = data.ICM20602Data.GyroZ.raw - data.ICM20602Data.GyroZ.zero;
	
	data.ICM20602Data.AccX.adjusted = data.ICM20602Data.AccX.raw;
	data.ICM20602Data.AccY.adjusted = data.ICM20602Data.AccY.raw;
	data.ICM20602Data.AccZ.adjusted = data.ICM20602Data.AccZ.raw;
	
	//30Hz   低通滤波
	//陀螺仪滤波
	Filters.GyroxLPF.input = data.ICM20602Data.GyroX.adjusted;
	Filters.GyroyLPF.input = data.ICM20602Data.GyroY.adjusted;
	Filters.GyrozLPF.input = data.ICM20602Data.GyroZ.adjusted;
	
	Butterworth30HzLPF(&Filters.GyroxLPF);
	Butterworth30HzLPF(&Filters.GyroyLPF);
	Butterworth30HzLPF(&Filters.GyrozLPF);
	
//	data.ICM20602Data.GyroX.filtered = Filters.GyroxLPF.output;
//	data.ICM20602Data.GyroY.filtered = Filters.GyroyLPF.output;
//	data.ICM20602Data.GyroZ.filtered = Filters.GyrozLPF.output;
	
	//加速度计滤波
	Filters.AccxLPF.input = data.ICM20602Data.AccX.adjusted;
	Filters.AccyLPF.input = data.ICM20602Data.AccY.adjusted;
	Filters.AcczLPF.input = data.ICM20602Data.AccZ.adjusted;
	
	Butterworth30HzLPF(&Filters.AccxLPF);
	Butterworth30HzLPF(&Filters.AccyLPF);
	Butterworth30HzLPF(&Filters.AcczLPF);
	
//	data.ICM20602Data.AccX.filtered = Filters.AccxLPF.output;
//	data.ICM20602Data.AccY.filtered = Filters.AccyLPF.output;
//	data.ICM20602Data.AccZ.filtered = Filters.AcczLPF.output;


	data.ICM20602Data.GyroX.filtered = data.ICM20602Data.GyroX.adjusted;
	data.ICM20602Data.GyroY.filtered = data.ICM20602Data.GyroY.adjusted;
	data.ICM20602Data.GyroZ.filtered = data.ICM20602Data.GyroZ.adjusted;
//	data.ICM20602Data.AccX.filtered = data.ICM20602Data.AccX.adjusted;
//	data.ICM20602Data.AccY.filtered = data.ICM20602Data.AccY.adjusted;
//	data.ICM20602Data.AccZ.filtered = data.ICM20602Data.AccZ.adjusted;
	data.ICM20602Data.AccX.filtered = Filters.AccxLPF.output;
	data.ICM20602Data.AccY.filtered = Filters.AccyLPF.output;
	data.ICM20602Data.AccZ.filtered = Filters.AcczLPF.output;

	//单位转换   弧度每秒
	data.ICM20602Data.GyroX.RadPerSec = data.ICM20602Data.GyroX.filtered * _gyro_scale;
	data.ICM20602Data.GyroY.RadPerSec = data.ICM20602Data.GyroY.filtered * _gyro_scale;
	data.ICM20602Data.GyroZ.RadPerSec = data.ICM20602Data.GyroZ.filtered * _gyro_scale;
	//单位转换   度每秒
	data.ICM20602Data.GyroX.DegPerSec = data.ICM20602Data.GyroX.RadPerSec * _RAD_TO_DEG ;
	data.ICM20602Data.GyroY.DegPerSec = data.ICM20602Data.GyroY.RadPerSec * _RAD_TO_DEG ;
	data.ICM20602Data.GyroZ.DegPerSec = data.ICM20602Data.GyroZ.RadPerSec * _RAD_TO_DEG ;
}

void get_gyro_zero(void){
	int i=0;
	float gyrox_add=0,gyroy_add=0,gyroz_add=0;short temp_gyro[3];
	for(i=0;i<500;i++)
	{
		icm20602_get_gyro_adc(&temp_gyro[0],&temp_gyro[1] ,&temp_gyro[2] );
		gyrox_add+=temp_gyro[0];gyroy_add+=temp_gyro[1];gyroz_add+=temp_gyro[2];
	}
	data.ICM20602Data.GyroX.zero = (float)(gyrox_add/500.0f);
	data.ICM20602Data.GyroY.zero = (float)(gyroy_add/500.0f);
	data.ICM20602Data.GyroZ.zero = (float)(gyroz_add/500.0f);
}

uint8_t icm20602_write_reg(uint8_t reg,uint8_t val)
{
	SPI1_Write_Byte(reg,val,CS_ICM_PORT,CS_ICM_IO);
	return 0;
}

uint8_t icm20602_read_reg(uint8_t reg)
{
	return SPI1_Read_Byte(reg,CS_ICM_PORT,CS_ICM_IO);  
}

uint8_t icm20602_read_buffer(uint8_t reg,void *buffer,uint8_t len)
{
	SPI1_Read_Len(reg,len,buffer,CS_ICM_PORT,CS_ICM_IO);  
	return 0;
}

uint8_t icm20602_init()
{
	uint8_t id;
	
	      HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//解锁
        HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;//确认
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//重新锁定
	
	X_GPIO_Output_Init(CS_ICM_PORT,CS_ICM_IO,CS_ICM_Peri_CLK);
	
	X_GPIO_Output_Init(ICM_Temp_Ctrl_PORT,ICM_Temp_Ctrl_IO,ICM_Temp_Ctrl_CLK);
	GPIO_L(ICM_Temp_Ctrl_PORT,ICM_Temp_Ctrl_IO);
	
	icm20602_write_reg(ICM20_PWR_MGMT_1,0x80);	//复位，复位后位0x41,睡眠模式，
	delay_ms(50);
	icm20602_write_reg(ICM20_PWR_MGMT_1,0x01);		//关闭睡眠，自动选择时钟
	delay_ms(50);
	
	id = icm20602_read_reg(ICM20_PWR_MGMT_1);//读取ID
	id = icm20602_read_reg(ICM20_WHO_AM_I);//读取ID
	printf("icm_20602 id=0x%x\r\n",id);
	
	if(id != 0x12)
	{
		printf("icm_20602 id error !!!\r\n");
		return 1;
	}
	printf("icm20602 init pass\r\n\r\n");
	
	icm20602_write_reg(ICM20_PWR_MGMT_2, 0x00);
	delay_ms(10);
	
	icm20602_write_reg(ICM20_SMPLRT_DIV,0);			//分频数=为0+1，数据输出速率为内部采样速率
	delay_ms(10);
	icm20602_write_reg(ICM20_CONFIG,DLPF_BW_20);	//GYRO低通滤波设置
	delay_ms(10);
	icm20602_write_reg(ICM20_ACCEL_CONFIG2,ACCEL_AVER_4|ACCEL_DLPF_BW_21);	//ACCEL低通滤波设置
	delay_ms(10);
	
	//设置量程
	icm20602_set_accel_fullscale(ICM20_ACCEL_FS_8G);
	delay_ms(10);
	icm20602_set_gyro_fullscale(ICM20_GYRO_FS_2000);
	delay_ms(10);
	
	icm20602_write_reg(ICM20_LP_MODE_CFG, 0x00);	//关闭低功耗
	delay_ms(10);
	icm20602_write_reg(ICM20_FIFO_EN, 0x00);		//关闭FIFO
	delay_ms(10);
	
	delay_ms(100);

	return 0;
}

//ICM20_ACCEL_FS_2G
//ICM20_ACCEL_FS_4G
//ICM20_ACCEL_FS_8G
//ICM20_ACCEL_FS_16G
uint8_t icm20602_set_accel_fullscale(uint8_t fs)
{
	switch(fs)
	{
		case ICM20_ACCEL_FS_2G:
			_accel_scale = 1.0f/16348.0f;
		break;
		case ICM20_ACCEL_FS_4G:
			_accel_scale = 1.0f/8192.0f;
		break;
		case ICM20_ACCEL_FS_8G:
			_accel_scale = 1.0f/4096.0f;
		break;
		case ICM20_ACCEL_FS_16G:
			_accel_scale = 1.0f/2048.0f;
		break;
		default:
			fs = ICM20_ACCEL_FS_8G;
			_accel_scale = 1.0f/4096.0f;
		break;

	}
	_accel_scale *= GRAVITY_MSS;
	return icm20602_write_reg(ICM20_ACCEL_CONFIG,fs);
}

//ICM20_GYRO_FS_250
//ICM20_GYRO_FS_500
//ICM20_GYRO_FS_1000
//ICM20_GYRO_FS_2000
uint8_t icm20602_set_gyro_fullscale(uint8_t fs)
{
	switch(fs)
	{
		case ICM20_GYRO_FS_250:
			_gyro_scale = 1.0f/131.068f;	//32767/250
		break;
		case ICM20_GYRO_FS_500:
			_gyro_scale = 1.0f/65.534f;
		break;
		case ICM20_GYRO_FS_1000:
			_gyro_scale = 1.0f/32.767f;
		break;
		case ICM20_GYRO_FS_2000:
			_gyro_scale = 1.0f/16.3835f;
		break;
		default:
			fs = ICM20_GYRO_FS_2000;
			_gyro_scale = 1.0f/16.3835f;
		break;

	}
	_gyro_scale *= _DEG_TO_RAD;
	return icm20602_write_reg(ICM20_GYRO_CONFIG,fs);
	
}

uint8_t icm20602_get_accel_adc(int16_t *accx,int16_t *accy,int16_t *accz)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_ACCEL_XOUT_H,buf,6))return 1;
	
	*accx = ((int16_t)buf[0]<<8) + buf[1];
	*accy = ((int16_t)buf[2]<<8) + buf[3];
	*accz = ((int16_t)buf[4]<<8) + buf[5];

	return 0;
}


uint8_t icm20602_get_gyro_adc(int16_t *gyrox,int16_t *gyroy,int16_t *gyroz)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_GYRO_XOUT_H,buf,6))return 1;
	*gyrox = (buf[0]<<8) + buf[1];
	*gyroy = (buf[2]<<8) + buf[3];
	*gyroz = (buf[4]<<8) + buf[5];

	return 0;
}

uint8_t icm20602_get_gyro(float *gyro)
{
	int16_t gyro_adc[3];
	if(icm20602_get_gyro_adc(&gyro_adc[0],&gyro_adc[1],&gyro_adc[2]))return 1;
	
	gyro[0] = _gyro_scale * gyro_adc[0];
	gyro[1] = _gyro_scale * gyro_adc[1];
	gyro[2] = _gyro_scale * gyro_adc[2];	

	return 0;
}

uint8_t icm20602_get_accel(float *accel)
{
	int16_t accel_adc[3];
	if(icm20602_get_accel_adc(&accel_adc[0],&accel_adc[1],&accel_adc[2]))return 1;
	accel[0] = _accel_scale * accel_adc[0];
	accel[1] = _accel_scale * accel_adc[1];
	accel[2] = _accel_scale * accel_adc[2];	
	return 0;
}

float icm20602_get_temp()
{
	int16_t temp_adc;
	uint8_t buf[2];
	if(icm20602_read_buffer(ICM20_TEMP_OUT_H,buf,2))return 0.0f;

	temp_adc = (buf[0]<<8)+buf[1];

	return (25.0f + (float)temp_adc/326.8f);
}
