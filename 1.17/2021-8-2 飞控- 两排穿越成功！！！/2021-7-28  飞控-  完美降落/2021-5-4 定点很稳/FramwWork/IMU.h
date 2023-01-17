#ifndef IMU_h
#define IMU_h
#include "includes.h"

extern float q0, q1, q2, q3;	// quaternion elements representing the estimated orientation


typedef struct
{
	float K1;// 一阶滤波 对加速度计取值的权重    K1 =0.05; 
	float dt;
	float Out;
	float Gyro;
	float AngleRaw;
}FilterOrder1TypeDef;

typedef struct
{
	float K2;// 二阶滤波对加速度计取值的权重    K2 =0.2; 
	float dt;
	float Out;
	float Gyro;
	float AngleRaw;
	float x1;//运算中间变量   
	float x2;//运算中间变量   
	float y1;//运算中间变量   
}FilterOrder2TypeDef;

typedef struct
{
	float dt;
	float gyroz;
	float mx;
	float my;
	float mz;
	float anglemx;
	float anglemy;
	float yawRAW;
	float yaw_Order1;
	float yaw_Order2;
	float yaw_f1;
	float yaw_f2;
	float yaw_f3;
}YawUpdateTypeDef;

extern YawUpdateTypeDef yaw_st;

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw);
void YawUpdate(YawUpdateTypeDef* pYaw);
void FilterOrder1Update(FilterOrder1TypeDef* pFilter);
void FilterOrder2Update(FilterOrder2TypeDef* pFilter);
void Quaternion_Adjust(float pitch,float roll,float yaw);
void Quaternion_Change(float tempYaw);

#endif
