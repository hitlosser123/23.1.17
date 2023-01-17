//=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

#include "AHRS.h"
#include <math.h>

//----------------------------------------------------------------------------------------------------
// Definitions

static float Kp =4.0f	;	//40	// proportional gain governs rate of convergence to accelerometer/magnetometer
static float  Ki= 0.003f	;	// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025f		// half the sample period

//---------------------------------------------------------------------------------------------------
// Variable definitions

float q0_AHRS = 1, q1_AHRS = 0, q2_AHRS = 0, q3_AHRS = 0;	// quaternion elements representing the estimated orientation
float exInt_AHRS = 0, eyInt_AHRS = 0, ezInt_AHRS = 0;	// scaled integral error

//====================================================================================================
// Function
//====================================================================================================

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, \
	                                float mx, float my, float mz,float angle[3]) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;

	// auxiliary variables to reduce number of repeated operations
	float q0q0 = q0_AHRS*q0_AHRS;
	float q0q1 = q0_AHRS*q1_AHRS;
	float q0q2 = q0_AHRS*q2_AHRS;
	float q0q3 = q0_AHRS*q3_AHRS;
	float q1q1 = q1_AHRS*q1_AHRS;
	float q1q2 = q1_AHRS*q2_AHRS;
	float q1q3 = q1_AHRS*q3_AHRS;
	float q2q2 = q2_AHRS*q2_AHRS;   
	float q2q3 = q2_AHRS*q3_AHRS;
	float q3q3 = q3_AHRS*q3_AHRS;          
	
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);       
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
	norm = sqrt(mx*mx + my*my + mz*mz);          
	mx = mx / norm;
	my = my / norm;
	mz = mz / norm;         
	
	// compute reference direction of flux
	hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;        
	
	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);  
	
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	
	// integral error scaled integral gain
	exInt_AHRS = exInt_AHRS + ex*Ki;
	eyInt_AHRS = eyInt_AHRS + ey*Ki;
	ezInt_AHRS = ezInt_AHRS + ez*Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt_AHRS;
	gy = gy + Kp*ey + eyInt_AHRS;
	gz = gz + Kp*ez + ezInt_AHRS;
	
	// integrate quaternion rate and normalise
	q0_AHRS = q0_AHRS + (-q1_AHRS*gx - q2_AHRS*gy - q3_AHRS*gz)*halfT;
	q1_AHRS = q1_AHRS + (q0_AHRS*gx + q2_AHRS*gz - q3_AHRS*gy)*halfT;
	q2_AHRS = q2_AHRS + (q0_AHRS*gy - q1_AHRS*gz + q3_AHRS*gx)*halfT;
	q3_AHRS = q3_AHRS + (q0_AHRS*gz + q1_AHRS*gy - q2_AHRS*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0_AHRS*q0_AHRS + q1_AHRS*q1_AHRS + q2_AHRS*q2_AHRS + q3_AHRS*q3_AHRS);
	q0_AHRS = q0_AHRS / norm;
	q1_AHRS = q1_AHRS / norm;
	q2_AHRS = q2_AHRS / norm;
	q3_AHRS = q3_AHRS / norm;
	
	
		angle[0] = asin(-2 * q1_AHRS * q3_AHRS + 2 * q0_AHRS* q2_AHRS)* 57.3;	// pitch
		angle[1] = atan2(2 * q2_AHRS * q3_AHRS + 2 * q0_AHRS * q1_AHRS, -2 * q1_AHRS * q1_AHRS - 2 * q2_AHRS* q2_AHRS + 1)* 57.3;	// roll
		angle[2] = atan2(2*(q1_AHRS*q2_AHRS + q0_AHRS*q3_AHRS),q0_AHRS*q0_AHRS+q1_AHRS*q1_AHRS-q2_AHRS*q2_AHRS-q3_AHRS*q3_AHRS) * 57.3;	//yaw
		
}

//====================================================================================================
// END OF CODE
//====================================================================================================
