#ifndef __ADRC_H
#define __ADRC_H	 
#include "includes.h"


typedef  unsigned short uint16 ;
typedef  short int16_t;
typedef  short int16;
typedef struct{
/*****安排过度过程*******/
	float x1;//跟踪微分期状态量
	float x2;//跟踪微分期状态量微分项
	float r;//时间尺度
	float h;//ADRC系统积分时间
	uint16 N0;//跟踪微分器解决速度超调h0=N*h 
	
	float h0;float fh;//最速微分加速度跟踪量
	
	/*****扩张状态观测器*******//******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
	float z1;
	float z2;
	float z3;//根据控制对象输入与输出，提取的扰动信息
	float e;//系统状态误差
	float y;//系统输出量
	float fe;
	float fe1;
	float beta_01;
	float beta_02;
	float beta_03;

	/**********系统状态误差反馈率*********/
	float e0;//状态误差积分项
	float e1;//状态偏差
	float e2;//状态量微分项
	float u0;//非线性组合系统输出
	float u;//带扰动补偿后的输出
	 
   /*********第一种组合形式*********/
	float beta_0;//线性
	float beta_1;//非线性组合参数
	float beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);
   
   /*********第二种组合形式*********/
	float alpha1;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
	float alpha2;//0<alpha1<1<alpha2
	float zeta;//线性段的区间长度
   
   /*********第三种组合形式*********/
	float h1;//u0=-fhan(e1,e2,r,h1);
	uint16 N1;//跟踪微分器解决速度超调h0=N*h
   
   /*********第四种组合形式*********/
	float c;//u0=-fhan(e1,c*e2*e2,r,h1);
	float b0;//扰动补偿
	
	float Up;
	float Ui;
	float Ud;
	
	float UpMax;
	float UiMax;
	float UdMax;
	
}Fhan_Data;


void ADRC_Init(Fhan_Data *fhan_Input1,Fhan_Data *fhan_Input2); 
void ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback_ADRC);
void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC);//安排ADRC过度过程
void ESO_ADRC(Fhan_Data *fhan_Input);
float Fal_ADRC(float e,float alpha,float zeta);//原点附近有连线性段的连续幂次函数
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
