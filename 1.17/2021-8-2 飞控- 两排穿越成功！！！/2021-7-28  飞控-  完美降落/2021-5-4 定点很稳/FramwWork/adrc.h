#ifndef __ADRC_H
#define __ADRC_H	 
#include "includes.h"


typedef  unsigned short uint16 ;
typedef  short int16_t;
typedef  short int16;
typedef struct{
/*****���Ź��ȹ���*******/
	float x1;//����΢����״̬��
	float x2;//����΢����״̬��΢����
	float r;//ʱ��߶�
	float h;//ADRCϵͳ����ʱ��
	uint16 N0;//����΢��������ٶȳ���h0=N*h 
	
	float h0;float fh;//����΢�ּ��ٶȸ�����
	
	/*****����״̬�۲���*******//******��ϵͳ���y������u�����ٹ���ϵͳ״̬���Ŷ�*****/
	float z1;
	float z2;
	float z3;//���ݿ��ƶ����������������ȡ���Ŷ���Ϣ
	float e;//ϵͳ״̬���
	float y;//ϵͳ�����
	float fe;
	float fe1;
	float beta_01;
	float beta_02;
	float beta_03;

	/**********ϵͳ״̬������*********/
	float e0;//״̬��������
	float e1;//״̬ƫ��
	float e2;//״̬��΢����
	float u0;//���������ϵͳ���
	float u;//���Ŷ�����������
	 
   /*********��һ�������ʽ*********/
	float beta_0;//����
	float beta_1;//��������ϲ���
	float beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);
   
   /*********�ڶ��������ʽ*********/
	float alpha1;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
	float alpha2;//0<alpha1<1<alpha2
	float zeta;//���Զε����䳤��
   
   /*********�����������ʽ*********/
	float h1;//u0=-fhan(e1,e2,r,h1);
	uint16 N1;//����΢��������ٶȳ���h0=N*h
   
   /*********�����������ʽ*********/
	float c;//u0=-fhan(e1,c*e2*e2,r,h1);
	float b0;//�Ŷ�����
	
	float Up;
	float Ui;
	float Ud;
	
	float UpMax;
	float UiMax;
	float UdMax;
	
}Fhan_Data;


void ADRC_Init(Fhan_Data *fhan_Input1,Fhan_Data *fhan_Input2); 
void ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback_ADRC);
void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC);//����ADRC���ȹ���
void ESO_ADRC(Fhan_Data *fhan_Input);
float Fal_ADRC(float e,float alpha,float zeta);//ԭ�㸽���������Զε������ݴκ���
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input);
float Constrain_Float(float amt, float low, float high); 
int16_t Sign_ADRC(float Input);
int16_t Fsg_ADRC(float x,float d);
//#define ABS(X)  (((X)>0)?(X):-(X))

extern	Fhan_Data ADRC_Pitch_Controller;
extern	Fhan_Data ADRC_Roll_Controller;

#include <stdio.h>
#include <math.h>

#endif
