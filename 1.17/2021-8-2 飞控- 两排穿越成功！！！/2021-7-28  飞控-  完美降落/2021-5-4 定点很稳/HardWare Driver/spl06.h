#ifndef _SP06_H_
#define _SP06_H_
#include "includes.h"

#define CS_SPL_PORT       GPIO_PORTC_BASE
#define CS_SPL_IO         GPIO_PIN_7
#define CS_SPL_Peri_CLK   SYSCTL_PERIPH_GPIOC


//ÆøÑ¹²âÁ¿ËÙÂÊ(sample/sec),Background Ä£Ê½Ê¹ÓÃ
#define  PM_RATE_1			(0<<4)		//1 measurements pr. sec.
#define  PM_RATE_2			(1<<4)		//2 measurements pr. sec.
#define  PM_RATE_4			(2<<4)		//4 measurements pr. sec.			
#define  PM_RATE_8			(3<<4)		//8 measurements pr. sec.
#define  PM_RATE_16			(4<<4)		//16 measurements pr. sec.
#define  PM_RATE_32			(5<<4)		//32 measurements pr. sec.
#define  PM_RATE_64			(6<<4)		//64 measurements pr. sec.
#define  PM_RATE_128		(7<<4)		//128 measurements pr. sec.

//ÆøÑ¹ÖØ²ÉÑùËÙÂÊ(times),Background Ä£Ê½Ê¹ÓÃ
#define PM_PRC_1			0		//Sigle			kP=524288	,3.6ms
#define PM_PRC_2			1		//2 times		kP=1572864	,5.2ms
#define PM_PRC_4			2		//4 times		kP=3670016	,8.4ms
#define PM_PRC_8			3		//8 times		kP=7864320	,14.8ms
#define PM_PRC_16			4		//16 times		kP=253952	,27.6ms
#define PM_PRC_32			5		//32 times		kP=516096	,53.2ms
#define PM_PRC_64			6		//64 times		kP=1040384	,104.4ms
#define PM_PRC_128			7		//128 times		kP=2088960	,206.8ms

//ÎÂ¶È²âÁ¿ËÙÂÊ(sample/sec),Background Ä£Ê½Ê¹ÓÃ
#define  TMP_RATE_1			(0<<4)		//1 measurements pr. sec.
#define  TMP_RATE_2			(1<<4)		//2 measurements pr. sec.
#define  TMP_RATE_4			(2<<4)		//4 measurements pr. sec.			
#define  TMP_RATE_8			(3<<4)		//8 measurements pr. sec.
#define  TMP_RATE_16		(4<<4)		//16 measurements pr. sec.
#define  TMP_RATE_32		(5<<4)		//32 measurements pr. sec.
#define  TMP_RATE_64		(6<<4)		//64 measurements pr. sec.
#define  TMP_RATE_128		(7<<4)		//128 measurements pr. sec.

//ÎÂ¶ÈÖØ²ÉÑùËÙÂÊ(times),Background Ä£Ê½Ê¹ÓÃ
#define TMP_PRC_1			0		//Sigle
#define TMP_PRC_2			1		//2 times
#define TMP_PRC_4			2		//4 times
#define TMP_PRC_8			3		//8 times
#define TMP_PRC_16			4		//16 times
#define TMP_PRC_32			5		//32 times
#define TMP_PRC_64			6		//64 times
#define TMP_PRC_128			7		//128 times

//SPL06_MEAS_CFG
#define MEAS_COEF_RDY		0x80
#define MEAS_SENSOR_RDY		0x40		//´«¸ÐÆ÷³õÊ¼»¯Íê³É
#define MEAS_TMP_RDY		0x20		//ÓÐÐÂµÄÎÂ¶ÈÊý¾Ý
#define MEAS_PRS_RDY		0x10		//ÓÐÐÂµÄÆøÑ¹Êý¾Ý

#define MEAS_CTRL_Standby				0x00		//¿ÕÏÐÄ£Ê½
#define MEAS_CTRL_PressMeasure			0x01	//µ¥´ÎÆøÑ¹²âÁ¿
#define MEAS_CTRL_TempMeasure			0x02	//µ¥´ÎÎÂ¶È²âÁ¿
#define MEAS_CTRL_ContinuousPress		0x05	//Á¬ÐøÆøÑ¹²âÁ¿
#define MEAS_CTRL_ContinuousTemp		0x06	//Á¬ÐøÎÂ¶È²âÁ¿
#define MEAS_CTRL_ContinuousPressTemp	0x07	//Á¬ÐøÆøÑ¹ÎÂ¶È²âÁ¿
//FIFO_STS
#define SPL06_FIFO_FULL		0x02
#define SPL06_FIFO_EMPTY	0x01
//INT_STS
#define SPL06_INT_FIFO_FULL		0x04
#define SPL06_INT_TMP			0x02
#define SPL06_INT_PRS			0x01
//CFG_REG
#define SPL06_CFG_T_SHIFT	0x08	//oversampling times>8Ê±±ØÐëÊ¹ÓÃ
#define SPL06_CFG_P_SHIFT	0x04

#define SP06_PSR_B2		0x00		//????
#define SP06_PSR_B1		0x01
#define SP06_PSR_B0		0x02
#define SP06_TMP_B2		0x03		//?¶??
#define SP06_TMP_B1		0x04
#define SP06_TMP_B0		0x05


#define SP06_PSR_CFG	0x06		//???????????
#define SP06_TMP_CFG	0x07		//?¶????????
#define SP06_MEAS_CFG	0x08		//????g?????

#define SP06_CFG_REG	0x09
#define SP06_INT_STS	0x0A
#define SP06_FIFO_STS	0x0B

#define SP06_RESET		0x0C
#define SP06_ID			0x0D

#define SP06_COEF		0x10		//-0x21,12?????
#define SP06_COEF_SRCE	0x28




uint8_t spl06_init(void);
void spl06_config_temperature(uint8_t rate,uint8_t oversampling);
void spl06_config_pressure(uint8_t rate,uint8_t oversampling);


void spl06_start(uint8_t mode);
int32_t spl06_get_pressure_adc(void);
int32_t spl06_get_temperature_adc(void);
void spl06_update(void);
float spl06_get_pressure(void);
float spl06_get_temperature(void);
float spl06_get_altitude(float pressure,float ground_pressure,float ground_temp);
#endif

