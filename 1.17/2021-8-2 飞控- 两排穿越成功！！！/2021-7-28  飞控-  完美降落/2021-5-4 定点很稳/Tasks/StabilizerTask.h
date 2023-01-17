#ifndef __STABILIZERTASK_H
#define __STABILIZERTASK_H	 
#include "config&param.h"
#include "delay.h"


#define case_Update_loc_Des          1
#define case_Update_v_loc_Des        2
#define case_Update_height_Des       3
#define case_Update_v_h_Des          4
#define case_Update_pitrol_Des       5
#define case_Update_yaw_Des          6
#define case_Update_gyro_Des         7


extern DataTypeDef data;
extern float Throttle_out,u_gyrox,u_gyroy,u_gyroz;
extern short Throttle_th;
extern float pitch_imu,roll_imu,yaw_imu;

extern UN_AIM_DATA     unAimData;
extern ST_VISION G_ST_Vision ;

void Compute_Motor(void);
void Update_Motor(void);
void StructureInit(void);
void Adjust_Yaw_Loop(void);
void Update_Data(void);
void Update_Des(unsigned char which_level);
void accel_to_lean_angles(float acc_tar_forward,float acc_tar_right,float *tar_pitch,float *tar_roll);//cm/s^2;

void stabilizerTask(void *pvParameters);



#endif
