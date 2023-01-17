#ifndef _AK8975_H_
#define	_AK8975_H_

#include "includes.h"

#define CS_AK_PORT       GPIO_PORTC_BASE
#define CS_AK_IO         GPIO_PIN_6
#define CS_AK_CLK        SYSCTL_PERIPH_GPIOC


/*
*测量时间:典型7.9ms，最大10ms
单位转换：最小0.285 典型0.3 最大0.315 μT/LSB
*/
#define AK8975_WIA     0x00		//Device ID = 0x48

#define AK8975_HXL     0x03
#define AK8975_HXH     0x04
#define AK8975_HYL     0x05
#define AK8975_HYH     0x06
#define AK8975_HZL     0x07
#define AK8975_HZH     0x08

#define AK8975_CNTL    0x0A


#define AK8975_INFO     0x01
#define AK8975_ST1     	0x02
#define AK8975_ST2     	0x09
#define AK8975_ASTC     0x0C
//#define AK8975_TS1     	0x0D
//#define AK8975_TS2     	0x0E
#define AK8975_I2CDIS   0x0F
#define AK8975_ASAX     0x10
#define AK8975_ASAY     0x11
#define AK8975_ASAZ     0x12


#define AK8975_MODE_PowerDown		0x00
#define AK8975_MODE_SignalMeasure	0x01
#define AK8975_MODE_SelfTest		0x08
#define AK8975_MODE_POWR_DOWN		0x0F


uint8_t ak8975_init(void);
uint8_t ak8975_start(void);
uint8_t ak8975_get_mag_adc(int16_t *mag);
uint8_t ak8975_get_mag(float *mag);
#endif

