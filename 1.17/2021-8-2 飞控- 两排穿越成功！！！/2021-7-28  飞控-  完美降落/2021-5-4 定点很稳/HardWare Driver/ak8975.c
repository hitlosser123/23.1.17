#include "ak8975.h"
#include "spi1.h"
#include "x_gpio.h"
static float _mag_scale;

uint8_t ak8975_write_reg(uint8_t reg,uint8_t val)
{
	SPI1_Write_Byte(reg,val,CS_AK_PORT,CS_AK_IO);
	return 0;
}
uint8_t ak8975_read_reg(uint8_t reg)
{
	return SPI1_Read_Byte(reg,CS_AK_PORT,CS_AK_IO);  
}

uint8_t ak8975_read_buffer(uint8_t reg,void *buffer,uint8_t len)
{
	SPI1_Read_Len(reg,len,buffer,CS_AK_PORT,CS_AK_IO);  
	return 0;
}


uint8_t ak8975_init()
{
	uint8_t id,info;
	
	X_GPIO_Output_Init(CS_AK_PORT,CS_AK_IO,CS_AK_CLK);
	
	id	= ak8975_read_reg(AK8975_WIA);
	UARTprintf("ak8975 id=0x%x\r\n",id);
//	while(id != 0x48)id	= ak8975_read_reg(AK8975_WIA);
	if(id != 0x48)
	{
		UARTprintf("ak8975 id error !!!\r\n");
		return 1;
	}
	info  = ak8975_read_reg(AK8975_INFO);
	UARTprintf("ak8975 info:0x%x\r\n",info);
	UARTprintf("ak8975 init pass\r\n\r\n");
	_mag_scale = 0.3f;
	
	
	ak8975_write_reg(AK8975_I2CDIS,0x1B);	//禁止IIC
	ak8975_start();	//启动一次测量
	return 0;
}

uint8_t ak8975_start()
{
	return ak8975_write_reg(AK8975_CNTL,AK8975_MODE_SignalMeasure);
}



uint8_t ak8975_get_mag_adc(int16_t *mag)
{
	uint8_t buf[6];
	
	if(ak8975_read_buffer(AK8975_HXL,buf,6))return 1;
	mag[0] = (buf[1]<<8) + buf[0];
	mag[1] = (buf[3]<<8) + buf[2];
	mag[2] = (buf[5]<<8) + buf[4];
	ak8975_start();		//重新启动测量
	return 0;
}


uint8_t ak8975_get_mag(float *mag)
{
	int16_t magadc[3];
	if(ak8975_get_mag_adc(magadc))return 1;
	
	mag[0] = _mag_scale * magadc[0];
	mag[1] = _mag_scale * magadc[1];
	mag[2] = _mag_scale * magadc[2];
	
	return 0;
}
