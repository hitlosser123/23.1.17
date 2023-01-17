#include "spl06.h"
#include "math.h"
#include "spi1.h"
#include "delay.h"
#include "x_gpio.h"

//	float groud_temp=25.0f,groud_press=100000.0f;		//
//	float pressure,temperature;
//	float baro_alt;								//unit m
//	spl06_update();
//			groud_press = spl06_get_pressure();
//			groud_temp = spl06_get_temperature();
//			spl06_update();				//Update DT=20ms
//	 		pressure = spl06_get_pressure();
//			temperature = spl06_get_temperature();
//			baro_alt = spl06_get_altitude(pressure,groud_press,groud_temp);		//alt now

uint8_t spl06_write_reg(uint8_t reg,uint8_t val)
{
	SPI1_Write_Byte(reg,val,CS_SPL_PORT,CS_SPL_IO);
	return 0;
}
uint8_t spl06_read_reg(uint8_t reg)
{	
	return SPI1_Read_Byte(reg,CS_SPL_PORT,CS_SPL_IO);  
}

uint8_t spl06_read_buffer(uint8_t reg,void *buffer,uint8_t len)
{
	SPI1_Read_Len(reg,len,buffer,CS_SPL_PORT,CS_SPL_IO);  
	return 0;
}


static int16_t _C0,_C1,_C01,_C11,_C20,_C21,_C30;
static int32_t _C00,_C10;

static float _kT,_kP;
static float _Temp,_Press;
static int32_t _raw_temp,_raw_press;

uint8_t spl06_init(void)
{
	uint8_t coef[18];
	uint8_t id;
	
	X_GPIO_Output_Init(CS_SPL_PORT,CS_SPL_IO,CS_SPL_Peri_CLK);
	
	
	if(spl06_write_reg(SP06_RESET,0x89))
	{
		puts("spl06 reset  fail\r\n");
		return 1;
	}
	
	id = spl06_read_reg(SP06_ID);
	printf("sol06 id=0x%x\r\n",id);//��ȡID
	
	if(id != 0x10)
	{
		printf("sol06 id error !!!\r\n");
	}
	
	delay_ms(200);
	spl06_read_buffer(SP06_COEF,coef,18);
	_C0 	= ((int16_t)coef[0]<<4 ) + ((coef[1]&0xF0)>>4);
	_C0 = (_C0&0x0800)?(0xF000|_C0):_C0;
	_C1 	= ((int16_t)(coef[1]&0x0F)<<8 ) + coef[2];
	_C1 = (_C1&0x0800)?(0xF000|_C1):_C1;
	

	_C00 	= ((int32_t)coef[3]<<12 ) + ((uint32_t)coef[4]<<4 ) + (coef[5]>>4);
	_C10   	= ((int32_t)(coef[5]&0x0F)<<16 ) + ((uint32_t)coef[6]<<8 ) + coef[7];
	_C00 = (_C00&0x080000)?(0xFFF00000|_C00):_C00;
	_C10 = (_C10&0x080000)?(0xFFF00000|_C10):_C10;
	
	
	_C01   	= ((int16_t)coef[8]<<8 ) + coef[9];
	_C11   	= ((int16_t)coef[10]<<8 ) + coef[11];
	_C20   	= ((int16_t)coef[12]<<8 ) + coef[13];
	_C21   	= ((int16_t)coef[14]<<8 ) + coef[15];
	_C30   	= ((int16_t)coef[16]<<8 ) + coef[17];

	
	spl06_config_pressure(PM_RATE_128,PM_PRC_64);
	spl06_config_temperature(PM_RATE_8,TMP_PRC_8);
	
	
	
	printf("_C0=%d\r\n",_C0);
	printf("_C1=%d\r\n",_C1);
	printf("_C00=%d\r\n",_C00);
	printf("_C10=%d\r\n",_C10);
	printf("_C01=%d\r\n",_C01);
	printf("_C11=%d\r\n",_C11);
	printf("_C20=%d\r\n",_C20);
	printf("_C21=%d\r\n",_C21);
	printf("_C30=%d\r\n",_C30);
	
	puts("spl06 init pass\r\n\r\n");
	
	spl06_start(MEAS_CTRL_ContinuousPressTemp);
	delay_ms(20);
	return 0;
}


void spl06_config_temperature(uint8_t rate,uint8_t oversampling)
{
	switch(oversampling)
	{
		case TMP_PRC_1:
			_kT = 524288;
			break;
		case TMP_PRC_2:
			_kT = 1572864;
			break;
		case TMP_PRC_4:
			_kT = 3670016;
			break;
		case TMP_PRC_8:
			_kT = 7864320;
			break;
		case TMP_PRC_16:
			_kT = 253952;
			break;
		case TMP_PRC_32:
			_kT = 516096;
			break;
		case TMP_PRC_64:
			_kT = 1040384;
			break;
		case TMP_PRC_128:
			_kT = 2088960;
			break;
	}
	_kT = 1.0f/_kT;
	
	spl06_write_reg(SP06_TMP_CFG,rate|oversampling|0x80);
	if(oversampling > TMP_PRC_8)
	{
		uint8_t temp = spl06_read_reg(SP06_CFG_REG);
		spl06_write_reg(SP06_CFG_REG,temp|SPL06_CFG_T_SHIFT);
	}
}

void spl06_config_pressure(uint8_t rate,uint8_t oversampling)
{
	switch(oversampling)
	{
		case PM_PRC_1:
			_kP = 524288;
			break;
		case PM_PRC_2:
			_kP = 1572864;
			break;
		case PM_PRC_4:
			_kP = 3670016;
			break;
		case PM_PRC_8:
			_kP = 7864320;
			break;
		case PM_PRC_16:
			_kP = 253952;
			break;
		case PM_PRC_32:
			_kP = 516096;
			break;
		case PM_PRC_64:
			_kP = 1040384;
			break;
		case PM_PRC_128:
			_kP = 2088960;
			break;
	}
	_kP = 1.0f/_kP;
	spl06_write_reg(SP06_PSR_CFG,rate|oversampling);
	if(oversampling > PM_PRC_8)
	{
		uint8_t temp = spl06_read_reg(SP06_CFG_REG);
		spl06_write_reg(SP06_CFG_REG,temp|SPL06_CFG_P_SHIFT);
	}
}

void spl06_start(uint8_t mode)
{
	spl06_write_reg(SP06_MEAS_CFG, mode);
}


int32_t spl06_get_pressure_adc()
{
	uint8_t buf[3];
	int32_t adc;
	spl06_read_buffer(SP06_PSR_B2,buf,3);
	adc = (int32_t)(buf[0]<<16) + (buf[1]<<8) + buf[2];
	adc = (adc&0x800000)?(0xFF000000|adc):adc;
	return adc;
}

int32_t spl06_get_temperature_adc()
{
	uint8_t buf[3];
	int32_t adc;
	spl06_read_buffer(SP06_TMP_B2,buf,3);
	adc = (int32_t)(buf[0]<<16) + (buf[1]<<8) + buf[2];
	adc = (adc&0x800000)?(0xFF000000|adc):adc;
	return adc;
}

void spl06_update_pressure()
{
	float Traw_src, Praw_src;
	float qua2, qua3;

	Traw_src = _kT * _raw_temp;
	Praw_src = _kP * _raw_press;

	_Temp = 0.5f*_C0 + Traw_src * _C1;

	qua2 = _C10 + Praw_src * (_C20 + Praw_src* _C30);
	qua3 = Traw_src * Praw_src * (_C11 + Praw_src * _C21);
	_Press = _C00 + Praw_src * qua2 + Traw_src * _C01 + qua3;
}


void spl06_update()
{
	_raw_temp = spl06_get_temperature_adc();
	_raw_press = spl06_get_pressure_adc();
	spl06_update_pressure();
}

float spl06_get_temperature()
{
	return _Temp;
}

float spl06_get_pressure()
{
	return _Press;
}

float spl06_get_altitude(float pressure,float ground_pressure,float ground_temp)
{
	float ret;
	float scaling = pressure / ground_pressure;
	float temp = ground_temp + 273.15f;

	// This is an exact calculation that is within +-2.5m of the standard atmosphere tables
	// in the troposphere (up to 11,000 m amsl).
	ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));

	return ret;
}
