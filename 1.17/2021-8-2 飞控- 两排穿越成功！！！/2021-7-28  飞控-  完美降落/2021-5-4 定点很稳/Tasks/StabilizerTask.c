#include "StabilizerTask.h"
#include "icm20602.h"
#include "sbus.h"
#include "AHRS.h"
#include "uart3_dma.h"
#include "uart2_dma.h"
#include "optical.h"
#include "height.h"
#include "RemoterTask.h"
#include "pid.h"
#include "us_100.h"
#include "spl06.h"
#include "icm20602.h"
#include "ak8975.h"
#include "mag.h"
#include "string.h"
#include "adrc.h"
#include "IMU.h"
#include "motors.h"
#include "openmv.h"
#include "SINS.h"
#include "anoV65.h"
#include "AutoflyTask.h"
#include "m1_pwm.h"

#define _DEG_TO_RAD    0.0174532f 							//��ת����


UN_AIM_DATA     unAimData;
ST_VISION G_ST_Vision={0};
UBaseType_t uxHighWaterMark;
DataTypeDef data;


float pitch_imu=0,roll_imu=0,yaw_imu=0;
float Throttle_out,u_gyrox,u_gyroy,u_gyroz;
short Throttle_th;
float ICM_Temp,ICM_Temp_KP=1.0f,ICM_Temp_U;
float AHRS_Angle[3];
void stabilizerTask(void *pvParameters)
{
	static u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt(); 
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );//�����ȥ��203�б�־
	vTaskDelay(1500);	/*��ʱ�ȴ��������ȶ�*/
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, CTRL_DT );		/*����������ʱ*/   //5ms
		
		Mag_Update();  ////��������,20ms 
		
		
	  pitch_imu = -unAimData.stEnemyE.Pitchangle ;
		roll_imu  = unAimData.stEnemyE.rollangle   ;
		yaw_imu   = -unAimData.stEnemyE.Yawangle   ;
		
		data.ICM20602Data.GyroX.DegPerSec  = -unAimData.stEnemyE.Gyro_X_Real;
		data.ICM20602Data.GyroY.DegPerSec   = unAimData.stEnemyE.Gyro_Y_Real;
		data.ICM20602Data.GyroZ.DegPerSec = -unAimData.stEnemyE.Gyro_Z_Real;
		
		data.ICM20602Data.GyroZ.RadPerSec =	data.ICM20602Data.GyroZ.DegPerSec *_DEG_TO_RAD ;
		data.ICM20602Data.GyroX.RadPerSec =	data.ICM20602Data.GyroX.DegPerSec *_DEG_TO_RAD ;
		data.ICM20602Data.GyroY.RadPerSec =	data.ICM20602Data.GyroY.DegPerSec *_DEG_TO_RAD ;
	  
		Height_Update();
		
		Check_Fly_Mode();

		Update_Data();
		
		
		Compute_Motor(); 
		Update_Motor(); 
		
		tick++;		
	}
}/*��ȡ����*//*Ŀ�����ݺͷ���ģʽ�趨*//*�������*//*�쳣���*//*PID����*//*���Ƶ�����*/


void Update_Motor(void)
{
		if(DroneStatus.ARM_Status==Armed)//������
		{
			if( DroneStatus.FlyMode == FlyMode_AttitudeHold )//����
			{
				if(Remoter.ThrCtrler>2040)
					Set_PWM_Motors();
				else Set_IDLE_Motors();//����Ϊ0�͵���
			}
			else if( DroneStatus.FlyMode == FlyMode_AltitudeHold ||//����
         			 DroneStatus.FlyMode == FlyMode_HorizontalRate ||  //
				       DroneStatus.FlyMode == FlyMode_PositionHold  ||  //����
			         DroneStatus.FlyMode == FlyMode_SDK   )//SDKģʽ
			{
				if(Ctrler.Z_posPID.FB < 0.2f && Remoter.ThrCtrler<2275)
				{
					Set_IDLE_Motors();//��û��ɣ���ء����أ���������ͣ�����
				}
				else if(SDK_DelayWakeFlag==1)  //SDK_DelayWakeFlag�����־λΪ1��ʱ��ᵡ��
				{
					Set_IDLE_Motors();
				}
				else 
				{
					Set_PWM_Motors();
				}
//				if(SDKLandFlag)Throttle_th=3180;//Set_Throttle_Motors();
//				else Throttle_th=Throttle_threshold; 
			}
			
			//!!!!!!!!!!!!!!!!!!!!!������ȼ�����������
			else if(DroneStatus.FlyMode == FlyMode_DangerousStop )//ǿ��ͣ��
			{
				Set_Zero_Motors();
				DroneStatus.ARM_Status = DisArmed;
			}
			else //�������elseӦ���Ǳ������ģʽ�ı����쳣��ʲôģʽҲ����
			{
				Set_Zero_Motors();
				DroneStatus.ARM_Status = DisArmed;
			}
			//!!!!!!!!!!!!!!!!   ELSE!!!!!!!!!!!!!!!!!!!
			
//			Set_Zero_Motors();
		}
		else//����״̬�������ֵ�����ҵ�����Բ�ת
		{
			Clear_Structure();
			Set_Zero_Motors();
		}
}

void StructureInit(void)
{
	Clear_Structure();
	PID_Init();
	DroneStatus.FlyMode = FlyMode_AttitudeHold;//��������ģʽ
	DroneStatus.ARM_Status=DisArmed;  
	mymotor.motor1=0;mymotor.motor2=0;mymotor.motor3=0;mymotor.motor4=0;
	
	Throttle_th=Throttle_threshold; 
	DroneStatus.PIDingDelayMS = 0;
	
	Ctrler.yawPID.Des=	0;//12.0;
	
	Ctrler.locxPID.Des=0;
	Ctrler.locyPID.Des=0;
}


void Update_Data(void){

	
	// x ��Ϊ��      y  ��Ϊ��
//	Ctrler.locxPID.FB= -stSINS.Position[1];	
//	Ctrler.locyPID.FB= -stSINS.Position[2];
//	

//	Ctrler.locxsPID.FB= -stSINS.Speed[1];
//	Ctrler.locysPID.FB= -stSINS.Speed[2]; 
	Ctrler.locxPID.FB=ano_of.DISTANCE_X ;
	Ctrler.locyPID.FB=ano_of.DISTANCE_Y ;
	
	Ctrler.locxsPID.FB= ano_of.of2_dx;
  Ctrler.locysPID.FB= ano_of.of2_dy;

	Ctrler.pitchPID.FB = pitch_imu ;
	Ctrler.rollPID.FB  = roll_imu ;
	Ctrler.yawPID.FB   = yaw_imu;


	Ctrler.gyroyPID.FB = -data.ICM20602Data.GyroY.DegPerSec ;
	Ctrler.gyroxPID.FB = -data.ICM20602Data.GyroX.DegPerSec;
	Ctrler.gyrozPID.FB = -data.ICM20602Data.GyroZ.DegPerSec;




//		Ctrler.Z_posPID.FB = data.SPL06Data.f2_h;
//		Ctrler.Z_ratePID.FB = data.SPL06Data.f2_v_h ;

		Ctrler.Z_posPID.FB = data.TofData.f2_h;
		Ctrler.Z_ratePID.FB = data.TofData.f2_v_h ;
		
//			Ctrler.Z_posPID.FB = stSINS.Position[0]/100.0f;//m
//		Ctrler.Z_ratePID.FB = stSINS.Speed[0]/100.0f ;	//m/s
	
}

unsigned char cnt_h,cnt_loc,cnt_locs,cnt_yaw;
void Compute_Motor(void){
	
	
	Update_Des(case_Update_height_Des);
	cnt_h++;
	if(cnt_h>=2)
	{
		ComputePID(&Ctrler.Z_posPID);
		cnt_h=0;
	}
	
   	Update_Des(case_Update_v_h_Des);
		SDK_Set_H_Loc();
	  ComputePID(&Ctrler.Z_ratePID);
	
	Update_Des(case_Update_loc_Des);
//	SDK_Set_Pos_Loc();
	cnt_loc++;
	if(cnt_loc>=2)
	{
		ComputePID(&Ctrler.locxPID);
		ComputePID(&Ctrler.locyPID);
		cnt_loc=0;
	}
	
	Update_Des(case_Update_v_loc_Des);
	SDK_Set_V_Loc();
//	cnt_locs++;
//	if(cnt_locs>=1)
//	{
		ComputePID(&Ctrler.locxsPID);
		ComputePID(&Ctrler.locysPID);
//		cnt_locs=0;
//	}
	
	
	Update_Des(case_Update_pitrol_Des);
	ComputePID(&Ctrler.pitchPID);
	ComputePID(&Ctrler.rollPID);
	Update_Des(case_Update_yaw_Des);
	
//	SDK_Set_Yaw();
	ComputeYawPID(&Ctrler.yawPID);
	
	
//	ADRC_Control(&ADRC_Pitch_Controller, Ctrler.gyroyPID.Des,Ctrler.gyroyPID.FB);
//	ADRC_Control(&ADRC_Roll_Controller,  Ctrler.gyroxPID.Des,Ctrler.gyroxPID.FB);
	
	Update_Des(case_Update_gyro_Des);
	
	SDK_Set_Gyroz();
	ComputePID(&Ctrler.gyroxPID);
	ComputePID(&Ctrler.gyroyPID);
	ComputePID(&Ctrler.gyrozPID);//Limit_PWM_add  z300


//Ԥ���߶�ͻ���һЩ˼·���Ҿ��û�����
//if(data.TofData.last_high == 1)
//{ 
//   if(Ctrler.Z_ratePID.U>130)
//		 Ctrler.Z_ratePID.U = 130;
//	 else if(Ctrler.Z_ratePID.U< -40)
//		 Ctrler.Z_ratePID.U = -40;
//	 else
//		  Ctrler.Z_ratePID.U=Ctrler.Z_ratePID.U;  
//}

    Throttle_th = 3310;  //��ͣ����3180    //����3180   �粻��3250
   	Throttle_out=Ctrler.Z_ratePID.U +  Throttle_th;
 
//		Throttle_out=Remoter.ThrCtrler;
		
		u_gyrox=Ctrler.gyroxPID.U ;
		u_gyroy=-Ctrler.gyroyPID.U;
		u_gyroz=-Ctrler.gyrozPID.U -200.0f;  //��������

//u_gyrox = u_gyroy =u_gyroz = 0;
	//u_gyrox = u_gyroy =u_gyroz =0;
	
 // u_gyroz = 0;
	
//	}

	mymotor.motor1= Throttle_out
									-u_gyroy//pitch
									-u_gyrox//roll
									+u_gyroz;//yaw
	
	mymotor.motor2= Throttle_out
									+u_gyroy//pitch
									+u_gyrox//roll
									+u_gyroz;//yaw
	
	mymotor.motor3= Throttle_out
									-u_gyroy//pitch
									+u_gyrox//roll
									-u_gyroz;//yaw
	
	mymotor.motor4= Throttle_out
									+u_gyroy//pitch
									-u_gyrox//roll
									-u_gyroz;//yaw  //   2018/7/2����yaw�߼�
	//		����У׼
//    mymotor.motor1= Throttle_out;
//		mymotor.motor2= Throttle_out;
//		mymotor.motor3= Throttle_out;
//		mymotor.motor4= Throttle_out;
  
			}

#define IS_low_Error(last,now)     (now-last<=2) && (now-last>=-2)
#define IS_Adjust_Yaw_Condition(last_yaw)   IS_low_Error(last_yaw,yaw_st.yaw_Order1)
unsigned char is_refreshed_yaw_des=0;
void Adjust_Yaw_Loop(void)
{
	static unsigned char time_cnt;
	static float Last_yaw;
	if( IS_Adjust_Yaw_Condition(Last_yaw) && pitch_imu>=-2 && \
									pitch_imu<=2 && roll_imu>=-2 && roll_imu<=2)
		time_cnt++;
	else time_cnt=0;
	Last_yaw = yaw_st.yaw_Order1;
	
	if(time_cnt>=200)
	{
		if(   (yaw_st.yaw_Order1-yaw_imu)>=2  ||   (yaw_st.yaw_Order1-yaw_imu)<=-2  )
		{
			if(is_refreshed_yaw_des==0)
			{
				Quaternion_Change(   yaw_st.yaw_Order1    );
//				Ctrler.yawPID.Des = yaw_st.yaw_Order1;
				is_refreshed_yaw_des=1;
			}
			else  Quaternion_Change( yaw_imu+(yaw_st.yaw_Order1-yaw_imu)*0.1f );
		}
		time_cnt=0;
	}
}

float mytest1 = 0;
float mytest2 = 0;
char flag_openmv_car = 0;

void Update_Des(unsigned char which_level)
{
  static unsigned char is_last_thr_valid,is_last_yaw_valid,is_last_pitch_valid,is_last_roll_valid;
	switch(which_level)
	{
		case case_Update_loc_Des://����λ������
			
			if( is_last_roll_valid && (!SBUS_CH_VALID(ROLL_CH)) )  //��һ�ζ��ˣ����ڻ���
			{
				Ctrler.locxPID.Des = Ctrler.locxPID.FB;
				mytest1++;
			}
			if( is_last_pitch_valid && (!SBUS_CH_VALID(PITCH_CH)) )
			{
				Ctrler.locyPID.Des = Ctrler.locyPID.FB;
				mytest2++;
			}
			
			is_last_pitch_valid = SBUS_CH_VALID(PITCH_CH);
			is_last_roll_valid = SBUS_CH_VALID(ROLL_CH);
			break;
			
    case case_Update_v_loc_Des://����ˮƽ�ٶ�����

				if(SBUS_CH_VALID(PITCH_CH))//��˶�Ӧ����ˮƽ�ٶ�
					   Ctrler.locysPID.Des = -((Remoter.PitCtrler-3000)/1000.0)*Stick_to_MAX_Horizontal_Rate;
				else if (Ctrler.locyPID.U>40)
						Ctrler.locysPID.Des = 40;
				else if (Ctrler.locyPID.U< -40)
						Ctrler.locysPID.Des = -40;
				else
					Ctrler.locysPID.Des = 	Ctrler.locyPID.U;//����˾Ͷ���
					
				if(SBUS_CH_VALID(ROLL_CH))//��˶�Ӧ����ˮƽ�ٶ�
					Ctrler.locxsPID.Des = -((Remoter.RolCtrler-3000)/1000.0)*Stick_to_MAX_Horizontal_Rate;  
				else if(Ctrler.locxPID.U>40)
					Ctrler.locxsPID.Des = 40;
				else if(Ctrler.locxPID.U< -40)
					Ctrler.locxsPID.Des = -40;
				else
					Ctrler.locxsPID.Des = 	Ctrler.locxPID.U;//����˾Ͷ���

       break;
				
		case case_Update_height_Des://���¸߶�����
			
			if(is_last_thr_valid && (!SBUS_THR_CH_VALID(THR_CH)) )
				Ctrler.Z_posPID.Des=  Ctrler.Z_posPID.FB;  //data.SPL06Data.f2_h;//Ҳ���Ը��µ�����ѹ

			is_last_thr_valid = SBUS_THR_CH_VALID(THR_CH);
//							Ctrler.Z_posPID.Des= 0.9f;
			
//			 flag_openmv_car = OpenMVData.BlobData.mag;
//			if(flag_openmv_car) //��ֹ�߶�ͻ��
//			{
//				 Ctrler.Z_posPID.Des=  Ctrler.Z_posPID.FB;
//				 Ctrler.Z_ratePID.FB = 0;
//			}
//			
       break;
			
		case case_Update_v_h_Des://������ֱ�ٶ�����
			if(SBUS_CH_VALID(THR_CH))
 				Ctrler.Z_ratePID.Des =  ((Remoter.ThrCtrler-3000)/1000.0)*Stick_to_MAX_V_height ;
			else
				Ctrler.Z_ratePID.Des = Ctrler.Z_posPID.U;
       break;
			
			
		case case_Update_pitrol_Des://���� pitch roll����

				accel_to_lean_angles( Ctrler.locysPID.U,-Ctrler.locxsPID.U,
			     	&Ctrler.pitchPID.Des,&Ctrler.rollPID.Des);

//						Ctrler.pitchPID.Des =  +Ctrler.locysPID.U;
//				    Ctrler.rollPID.Des = -Ctrler.locxsPID.U;
	
//				Ctrler.pitchPID.Des = -((Remoter.PitCtrler-3000)/1000.0)*Stick_to_MAX_Angle ;//-0.35;
//				Ctrler.rollPID.Des  = ((Remoter.RolCtrler-3000)/1000.0)*Stick_to_MAX_Angle ;//-0.15;   

     break;
			
		case case_Update_yaw_Des://����yaw����
			if(is_last_yaw_valid && (!SBUS_CH_VALID(YAW_CH)) )
			Ctrler.yawPID.Des = Ctrler.yawPID.FB;
			
			is_last_yaw_valid = SBUS_CH_VALID(YAW_CH);
		

       break;
			
		case case_Update_gyro_Des://���½��ٶ�����

			Ctrler.gyroyPID.Des = -Ctrler.pitchPID.U ;
			Ctrler.gyroxPID.Des = Ctrler.rollPID.U ;

	
			if(SBUS_CH_VALID(YAW_CH))
				Ctrler.gyrozPID.Des =  ((Remoter.YawCtrler-3000)/1000.0)*Stick_to_MAX_GyroZ ;
			else
//				 Ctrler.gyrozPID.Des = 0;
				Ctrler.gyrozPID.Des = Ctrler.yawPID.U ;
			
		
       break;
			
    default: 
       break;
	}
}


// a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
float fast_atan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f+v2*0.4378497304f)/(1.6867633134f+v2));
}
/// accel_to_lean_angles - horizontal desired acceleration to lean angles
///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
void accel_to_lean_angles(float acc_tar_forward,float acc_tar_right,float *tar_pitch,float *tar_roll)//cm/s^2
{
  float lean_angle_max = 30;
	
	float Cos_Roll;
	Cos_Roll = cos(roll_imu*DEG2RAD);//*Cos_Roll
	
  // update angle targets that will be passed to stabilize controller
  *tar_pitch=Constrain_Float(
									fast_atan(    acc_tar_forward    *Cos_Roll   /(GRAVITY_MSS*100)    )*RAD2DEG,
														-lean_angle_max,lean_angle_max);//pitch
  *tar_roll = Constrain_Float(
									fast_atan(acc_tar_right/(GRAVITY_MSS*100))*RAD2DEG,
														-lean_angle_max,lean_angle_max);//roll
}

