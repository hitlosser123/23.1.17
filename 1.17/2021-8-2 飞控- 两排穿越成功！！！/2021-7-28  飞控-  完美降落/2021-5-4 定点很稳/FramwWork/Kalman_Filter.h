#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H	 
#include "includes.h"

typedef struct 
{
	float angle;
	float angle_dot;//�������˲�������ǶȺͽ��ٶ�    

	float angle_0;
	float angle_dot_0;//�ɼ����ĽǶȺͽ��ٶ�    

	float P[2][2];    //�����м����      = {{ 1, 0 },  { 0, 1 }}
	float Pdot[4];// ={ 0,0,0,0};    
	float Q_angle;//=0.001;
	float Q_gyro;//=;//0.005; //�Ƕ��������Ŷ�,���ٶ��������Ŷ�    
	float R_angle;//=0.5;
	float C_0;// = 1;      
	float q_bias;
	float angle_err;
	float PCt_0;
	float PCt_1;
	float E;
	float K_0;
	float K_1;
	float t_0;
	float t_1; 
	float dt;//��������
}Kalman_FilterTypeDef;


void Kalman_Filter_Init(Kalman_FilterTypeDef* Kalman_Filter);

void Kalman_Filter(double angle_m,double gyro_m,Kalman_FilterTypeDef* Kalman_Filter);
void getangle(void);

extern Kalman_FilterTypeDef Pitch_Kalman_Filter;
extern Kalman_FilterTypeDef Roll_Kalman_Filter;

#endif
