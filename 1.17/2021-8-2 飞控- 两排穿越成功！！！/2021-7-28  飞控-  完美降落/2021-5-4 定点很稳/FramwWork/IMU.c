// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer 
// units are irrelevant as the vector is normalised.
#include "IMU.h"
#include <math.h>
#include "StabilizerTask.h"

//0.1          0.0001
//  float Kp= 4.0f	;		// proportional gain governs rate of convergence to accelerometer/magnetometer
//  float  Ki= 0.005f;		// integral gain governs rate of convergence of gyroscope biases
	
	  float Kp= 0.1f	;		// proportional gain governs rate of convergence to accelerometer/magnetometer
  float  Ki= 0.0001f;		// integral gain governs rate of convergence of gyroscope biases
	
#define halfT  (0.5f*CTRL_DT/1000.0f)		// half the sample period

#define q30  1073741824.0f  //q30格式,long转float时的除数.

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;	// quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;	// scaled integral error

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;         
	
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);       
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;      
	
	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	// integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
//			q0 = q0 / q30;	//q30格式转换为浮点数
//		q1 = q1 / q30;
//		q2 = q2 / q30;
//		q3 = q3 / q30; 
		//计算得到俯仰角/横滚角/航向角
		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.2928f;	// pitch
		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.2928f;	// roll
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.2928f;	//yaw
		
}


void FilterOrder1Update(FilterOrder1TypeDef* pFilter)
{
	pFilter->Out = pFilter->K1 * pFilter->AngleRaw +   \
								( 1 - pFilter->K1 ) * ( pFilter->Out + pFilter->Gyro * pFilter->dt );
}

void FilterOrder2Update(FilterOrder2TypeDef* pFilter)
{
	pFilter->x1 = ( pFilter->AngleRaw - pFilter->Out )*(1-pFilter->K2)*(1-pFilter->K2);    
  pFilter->y1 = pFilter->y1 + pFilter->x1 * pFilter->dt;    
  pFilter->x2 = pFilter->y1 + 2*(1-pFilter->K2)*(pFilter->AngleRaw-pFilter->Out)+pFilter->Gyro;    
  pFilter->Out= pFilter->Out+ pFilter->x2 * pFilter->dt;    
}

FilterOrder1TypeDef F1St={0.05,0.005,0,0,0};
FilterOrder2TypeDef F2St={0.2, 0.005,0,0,0,0,0,0};
YawUpdateTypeDef yaw_st;
void YawUpdate(YawUpdateTypeDef* pYaw)
{
	pYaw->yawRAW = atan2( pYaw->my, pYaw->mx)*57.3;
		 //if(angle_AHRS[0]>0)angle_AHRS[1]=angle_AHRS[0];else angle_AHRS[1]=angle_AHRS[0]+360.0;
	pYaw->yaw_f1 = pYaw->yaw_f1 * 0.9f + pYaw->yawRAW * 0.1f;
	pYaw->yaw_f2 = pYaw->yaw_f2 * 0.9f + pYaw->yaw_f1 * 0.1f;
	pYaw->yaw_f3 = pYaw->yaw_f3 * 0.9f + pYaw->yaw_f2 * 0.1f;
	
	F1St.AngleRaw = pYaw->yawRAW;
	F2St.AngleRaw = pYaw->yawRAW;
	F1St.Gyro = pYaw->gyroz;
	F2St.Gyro = pYaw->gyroz;
	FilterOrder1Update(&F1St);
	FilterOrder2Update(&F2St);
	pYaw->yaw_Order1 = F1St.Out;
	pYaw->yaw_Order2 = F2St.Out;
}

float _q0,_q1,_q2,_q3;
void Quaternion_Adjust(float pitch,float roll,float yaw)
{
	float CosHalfP,SinHalfP,CosHalfR,SinHalfR,CosHalfY,SinHalfY;
	float RadHalfP,RadHalfR,RadHalfY;
	float norm;
	RadHalfP = pitch/2.0f/57.3f;
	RadHalfR = roll/2.0f/57.3f;
	RadHalfY = yaw/2.0f/57.3f;
	CosHalfP = cos(RadHalfP);
	SinHalfP = sin(RadHalfP);
	CosHalfR = cos(RadHalfR);
	SinHalfR = sin(RadHalfR);
	CosHalfY = cos(RadHalfY);
	SinHalfY = sin(RadHalfY);
	
	_q0 = CosHalfY * CosHalfP * CosHalfR - SinHalfY * SinHalfP *SinHalfR;
	_q1 = CosHalfY * CosHalfP * SinHalfR + SinHalfY * SinHalfP *CosHalfR;
	_q2 = CosHalfY * SinHalfP * CosHalfR - SinHalfY * CosHalfP *SinHalfR;
	_q3 = SinHalfY * CosHalfP * CosHalfR + CosHalfY * SinHalfP *SinHalfR;
	
		// normalise quaternion
	norm = sqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
	_q0 = _q0 / norm;
	_q1 = _q1 / norm;
	_q2 = _q2 / norm;
	_q3 = _q3 / norm;
}

void Quaternion_Change(float tempYaw)
{
	Quaternion_Adjust(pitch_imu,roll_imu,tempYaw);
	q0 = _q0;
	q1 = _q1;
	q2 = _q2;
	q3 = _q3;
}
