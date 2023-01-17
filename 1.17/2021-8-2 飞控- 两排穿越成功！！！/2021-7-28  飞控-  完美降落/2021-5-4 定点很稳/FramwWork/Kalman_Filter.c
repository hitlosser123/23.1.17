#include "Kalman_Filter.h"
#include "config&param.h"
#include <math.h>


float dt = (float)CTRL_DT/1000.0f;/*0.02second*/   
float angleAx,gyroGy,angleAy,gyroGx;//�����ĽǶȣ���x��нǣ��ͽ��ٶ�      
short ax, ay, az, gx, gy, gz;//������ԭʼ���� 

Kalman_FilterTypeDef Pitch_Kalman_Filter;
Kalman_FilterTypeDef Roll_Kalman_Filter;
void getangle(void)     
{ 
//	ax=-sensors_accx;
//	ay=sensors_accy;
//	az=sensors_accz;   
    angleAx=atan2(ax,az)*180/3.1415926;//������x��н�    
		angleAy=atan2(ay,az)*180/3.1415926;
	
//	data.gyroxdata.stdunit = ( sensors_gyrox - data.gyroxdata.zero)/MPU_Range;//������ٶ� 
//  data.gyroydata.stdunit = ( sensors_gyroy - data.gyroydata.zero)/MPU_Range;//������ٶ� 
//  gyroGy = data.gyroydata.stdunit ;
//	gyroGx = data.gyroxdata.stdunit ;
   // Kalman_Filter(angleAx,gyroGy,angleAy,gyroGx);   //�������˲�    
}    
        
void Kalman_Filter(double angle_m,double gyro_m,Kalman_FilterTypeDef* Kalman_Filter) //angleAx �� gyroGy       
{    
	Kalman_Filter->angle += (gyro_m - Kalman_Filter->q_bias) * Kalman_Filter->dt;    
	Kalman_Filter->angle_err = angle_m - Kalman_Filter->angle;    
	Kalman_Filter->Pdot[0] = Kalman_Filter->Q_angle - Kalman_Filter->P[0][1] - Kalman_Filter->P[1][0];
	Kalman_Filter->Pdot[1] = - Kalman_Filter->P[1][1];    
	Kalman_Filter->Pdot[2] = - Kalman_Filter->P[1][1];    								
	Kalman_Filter->Pdot[3] = Kalman_Filter->Q_gyro;    
	Kalman_Filter->P[0][0] += Kalman_Filter->Pdot[0] * Kalman_Filter->dt;    
	Kalman_Filter->P[0][1] += Kalman_Filter->Pdot[1] * Kalman_Filter->dt;    
	Kalman_Filter->P[1][0] += Kalman_Filter->Pdot[2] * Kalman_Filter->dt;    
	Kalman_Filter->P[1][1] += Kalman_Filter->Pdot[3] * Kalman_Filter->dt;    
	Kalman_Filter->PCt_0 = Kalman_Filter->C_0 * Kalman_Filter->P[0][0];    
	Kalman_Filter->PCt_1 = Kalman_Filter->C_0 * Kalman_Filter->P[1][0];    
	Kalman_Filter->E = Kalman_Filter->R_angle + Kalman_Filter->C_0 * Kalman_Filter->PCt_0;    
	Kalman_Filter->K_0 = Kalman_Filter->PCt_0 / Kalman_Filter->E;    
	Kalman_Filter->K_1 = Kalman_Filter->PCt_1 / Kalman_Filter->E;    
	Kalman_Filter->t_0 = Kalman_Filter->PCt_0;    
	Kalman_Filter->t_1 = Kalman_Filter->C_0 * Kalman_Filter->P[0][1];    
	Kalman_Filter->P[0][0] -= Kalman_Filter->K_0 * Kalman_Filter->t_0;    
	Kalman_Filter->P[0][1] -= Kalman_Filter->K_0 * Kalman_Filter->t_1;    
	Kalman_Filter->P[1][0] -= Kalman_Filter->K_1 * Kalman_Filter->t_0;    
	Kalman_Filter->P[1][1] -= Kalman_Filter->K_1 * Kalman_Filter->t_1;    
	Kalman_Filter->angle += Kalman_Filter->K_0 * Kalman_Filter->angle_err; //���ŽǶ�    
	Kalman_Filter->q_bias += Kalman_Filter->K_1 * Kalman_Filter->angle_err;    
	Kalman_Filter->angle_dot = gyro_m-Kalman_Filter->q_bias;//���Ž��ٶ�    	
}    

void Kalman_Filter_Init(Kalman_FilterTypeDef* Kalman_Filter)
{
	Kalman_Filter->C_0=1;
	Kalman_Filter->R_angle=0.5;
	Kalman_Filter->Q_gyro=0.005;
	Kalman_Filter->Q_angle=0.001;
	Kalman_Filter->P[0][0]=1;
	Kalman_Filter->P[0][1]=0;
	Kalman_Filter->P[1][0]=0;
	Kalman_Filter->P[1][1]=1;
	
	Kalman_Filter->dt=0.001;
}
