#include "includes.h"
#include "mag.h"
#include "StabilizerTask.h"
#include "ak8975.h"
#include "SensorsTask.h"

#define F330
#ifdef  X215
#define MAG0_MIN     -38.0f
#define	MAG0_MAX	    -7.0f
#define MAG1_MIN		 -28.0f
#define MAG1_MAX      5.0f
#define MAG2_MIN		 -57.9f
#define MAG2_MAX      18.6f

#else
#ifdef F330
#define MAG0_MIN     -28.5f
#define	MAG0_MAX	     9.6f
#define MAG1_MIN		  -3.6f
#define MAG1_MAX      32.4f
#define MAG2_MIN		  32.7f
#define MAG2_MAX      56.7f
#endif
#endif
MagCelTypeDef  M_CelST;
//float mag_0,mag_1,mag_2;
//		 mag_0=mag[0]+25.0;//+6   -22.5   //		 -61   13
//		 mag_1=mag[1]+31.0;//-17   -25   //		 -70   7
//		 mag_2=mag[2]+27.0;//		 -66   11
													
void Mag_Update(void)
{
	ak8975_get_mag(mag);		//´ÅÁ¦¸üÐÂ,20ms 
	data.AK8975Data.MagX_raw = mag[0];
	data.AK8975Data.MagY_raw = mag[1];
	data.AK8975Data.MagZ_raw = mag[2];
	
	Mag_Adjust(); 
}

void Mag_Adjust(void)
{
	data.AK8975Data.MagX_Adjusted = data.AK8975Data.MagX_raw - M_CelST.X_offset ;
	data.AK8975Data.MagY_Adjusted = (data.AK8975Data.MagY_raw - M_CelST.Y_offset)*M_CelST.MAG1_SCALE ;
	data.AK8975Data.MagZ_Adjusted = (data.AK8975Data.MagZ_raw - M_CelST.Z_offset)*M_CelST.MAG2_SCALE ;
}

#define Mag_Calibration_OK   1
void Mag_Calibration_Loop (void)
{
	static unsigned char Mag_Calibration_Status;
	static float temp_Mx_max,temp_Mx_min,temp_My_max,temp_My_min,temp_Mz_max,temp_Mz_min;
	
	if(pitch_imu>=-2 && pitch_imu<=2 && roll_imu>=-2 && roll_imu<=2 )
	{
		if(mag[0]>temp_Mx_max)temp_Mx_max=mag[0];
		if(mag[0]<temp_Mx_min)temp_Mx_min=mag[0];
		if(mag[1]>temp_My_max)temp_My_max=mag[1];
		if(mag[1]<temp_My_min)temp_My_min=mag[1];
		if(mag[2]>temp_Mz_max)temp_Mz_max=mag[2];
		if(mag[2]<temp_Mz_min)temp_Mz_min=mag[2];
	}
	
	if(Mag_Calibration_Status==Mag_Calibration_OK)
	{
		Mag_Save_Adjust_Data(temp_Mx_max,temp_Mx_min,temp_My_max,temp_My_min,temp_Mz_max,temp_Mz_min);
	}
	
}

void Mag_ST_Init(void)
{
	M_CelST.X_max = MAG0_MAX ;
	M_CelST.X_min = MAG0_MIN ;
	M_CelST.Y_max = MAG1_MAX ;
	M_CelST.Y_min = MAG1_MIN ;
	M_CelST.Z_max = MAG2_MAX ;
	M_CelST.Z_min = MAG2_MIN ;

	M_CelST.X_offset = ( M_CelST.X_max + M_CelST.X_min )/2.0f;
	M_CelST.Y_offset = ( M_CelST.Y_max + M_CelST.Y_min )/2.0f;
	M_CelST.Z_offset = ( M_CelST.Z_max + M_CelST.Z_min )/2.0f;
	
	M_CelST.MAG1_SCALE = (M_CelST.X_max-M_CelST.X_min)/(M_CelST.Y_max-M_CelST.Y_min);
	M_CelST.MAG2_SCALE = (M_CelST.X_max-M_CelST.X_min)/(M_CelST.Z_max-M_CelST.Z_min);
}

void Mag_Save_Adjust_Data(float xMax,float xMin,float yMax,float yMin,float zMax,float zMin)
{
	M_CelST.X_max = xMax ;
	M_CelST.X_min = xMin ;
	M_CelST.Y_max = yMax ;
	M_CelST.Y_min = yMin ;
	M_CelST.Z_max = zMax ;
	M_CelST.Z_min = zMin ;

	M_CelST.X_offset = ( M_CelST.X_max + M_CelST.X_min )/2.0f;
	M_CelST.Y_offset = ( M_CelST.Y_max + M_CelST.Y_min )/2.0f;
	M_CelST.Z_offset = ( M_CelST.Z_max + M_CelST.Z_min )/2.0f;
	
	M_CelST.MAG1_SCALE = (M_CelST.X_max-M_CelST.X_min)/(M_CelST.Y_max-M_CelST.Y_min);
	M_CelST.MAG2_SCALE = (M_CelST.X_max-M_CelST.X_min)/(M_CelST.Z_max-M_CelST.Z_min);
	
	//EEPROM??????????
	//MAX  MIN
}
