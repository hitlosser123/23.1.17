#ifndef __OPTICAL_H
#define __OPTICAL_H
#include "includes.h"
#include "uart1_dma.h"


//==����/����
typedef unsigned char  	u8;/* defined for unsigned 8-bits integer variable 	  �޷���8λ���ͱ���  */
typedef  char   	      s8;/* defined for unsigned 8-bits integer variable 	  �޷���8λ���ͱ���  */
typedef short         	s16;/* defined for unsigned 8-bits integer variable 	  �޷���8λ���ͱ���  */

typedef struct
{
	//
	u8 of_update_cnt;//�������ݸ��¼�����
	u8 alt_update_cnt;//�߶����ݸ��¼�����
	//
	u8 link_sta;//����״̬��0��δ���ӡ�1�������ӡ�
	u8 work_sta;//����״̬��0���쳣��1������
	//
	u8 of_quality;
	//
	u8 of0_sta;
	s8 of0_dx;
	s8 of0_dy;
	//
	u8 of1_sta;
	s16 of1_dx;
	s16 of1_dy;	
	//
	u8 of2_sta;
	s16 of2_dx;
	s16 of2_dy;	
	s16 of2_dx_fix;
	s16 of2_dy_fix;
	s16 intergral_x;
	s16 intergral_y;
	//
	u32 of_alt_cm;
	//
	float quaternion[4];
	//
	s16 acc_data_x;
	s16 acc_data_y;
	s16 acc_data_z;
	s16 gyr_data_x;
	s16 gyr_data_y;
	s16 gyr_data_z;
	float DISTANCE_X;    //������֣���λ cm,(-32768~+32767 ѭ��)
	float DISTANCE_Y;    //������֣���λ cm,(-32768~+32767 ѭ��)
	float TAR_POSITION_X;
	float TAR_POSITION_Y;
	

}_ano_of_st;

//�ɿ�״̬


//==��������
extern _ano_of_st ano_of;
//==��������
//static
static void AnoOF_DataAnl(uint8_t *data_buf,uint8_t num);

//public
void AnoOF_GetOneByte(uint8_t data);
void AnoOF_Check_State(float dT_s);

extern void AnoOF_GetOneByte(uint8_t data);

#endif

