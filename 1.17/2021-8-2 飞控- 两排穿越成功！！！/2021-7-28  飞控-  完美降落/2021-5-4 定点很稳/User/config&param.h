#ifndef __MY_CONFIG_H
#define __MY_CONFIG_H

/********宏定义区*********/

#define Stick_to_MAX_Angle   15.0  //pitch roll 正负15度
#define Stick_to_MAX_Horizontal_Rate   100.0    //   cm/s
#define Stick_to_MAX_GyroZ            80.0   //  deg/s
#define Stick_to_MAX_V_height         0.6   //   m/s


#define CTRL_DT           5    //ms
#define DATA_Upload_DT   20
#define LED_DT  20
#define to_report_number   1

#define  MPU6050_SAMP_RATE   500
#define MPU6050_PERIOD      1000/MPU6050_SAMP_RATE
//卡尔曼滤波dt     量程
//数据更新，量程

#define MPU_Range   16.384


#define ARM_Delay_time  150
#define DISARM_Delay_time  50// 50*20ms = 1s

#define   value_limit(x,small,big)   if(x<small)x=small;if(x>big)x=big;

#define Remoter_ON    	Cap_Number[1]>1990 &&  Cap_Number[1]<4010 &&\
												Cap_Number[2]>1990 &&  Cap_Number[2]<4010 &&\
												Cap_Number[3]>1990 &&  Cap_Number[3]<4010 &&\
												Cap_Number[4]>1990 &&  Cap_Number[4]<4010  

#define offset_prio configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY//优先级低一点，不然RTOS管理不了，为了兼容原函数

#define sensorsTask_Prio      6
#define stabilizerTask_Prio   5
#define datalinkTask_Prio     4
#define remoter_task_Prio     4
#define autofly_task_Prio     4





#define Limit_PWM_add 500
#define Integral_limit  500
#define Integral_limit_H  2.5


#define THROTTLE_FENDUAN    2800


#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define MAIN_LOOP_RATE 	RATE_50_HZ
#define MAIN_LOOP_DT	(u32)(1000/MAIN_LOOP_RATE)	/*单位ms*/

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)


#define FlyMode_DangerousStop        0
#define FlyMode_AttitudeHold         1     //自稳模式
#define FlyMode_AltitudeHold         2     //定高模式
#define FlyMode_HorizontalRate       3     //水平速度闭环
#define FlyMode_PositionHold         4     //定点模式
#define FlyMode_SDK                  5     //SDK模式



#define get_gyro_BIT	  ( 1 << 0 )
#define get_angle_BIT 	( 1 << 1 )
#define get_height_BIT	( 1 << 2 )
#define got_channel1_BIT	( 1 << 3 )
#define got_channel2_BIT	( 1 << 4 )
#define got_channel3_BIT	( 1 << 5 )
#define got_channel4_BIT	( 1 << 6 )
#define get_flow_BIT	( 1 << 7 )
#define got_Uart2_RX_BIT  ( 1 << 8 )
#define got_SBUS_BIT	( 1 << 9 )
#define got_SONAR_BIT	( 1 << 10 )



#define ABS(x)   ( (x)>0?(x):-(x) ) 

#define   SBUS_OFFSET    100
#define   SBUS_THR_OFFSET    100
#define   SBUS_MID       1024
#define   SBUS_MAX       1696
#define   SBUS_MIN       352
#define   SBUS_CH_VALID(x)      ( ABS(x-SBUS_MID)>SBUS_OFFSET  )
#define   SBUS_THR_CH_VALID(x)      ( ABS(x-SBUS_MID)>SBUS_THR_OFFSET  )


#define   PI        3.14159f
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
// acceleration due to gravity in m/s/s
#define GRAVITY_MSS 9.80665f




typedef struct 
{
  float Des;//控制变量目标值
  float FB;//控制变量反馈值
	
	float Kp;//比例系数Kp
	float Ki;//积分系数Ki
	float Kd;//微分系数Kd
	
	float Up;//比例输出
	float Ui;//积分输出
	float Ud;//微分输出
	
	float E;//本次偏差
	float PreE;//上次偏差
  float SumE;//总偏差
	float U;//本次PID运算结果
	
	float UMax;//PID运算后输出最大值及做遇限削弱时的上限值
	float UpMax;//比例项输出最大值
	float UiMax;//积分项输出最大值
	float UdMax;//微分项输出最大值
	float SumEMax;//积分饱和值
	float EMin;//积分分离阈值
}PIDTypeDef;

typedef struct 
{
	PIDTypeDef    pitchPID;
	PIDTypeDef    rollPID;
	PIDTypeDef    yawPID;
	PIDTypeDef    gyroxPID;
	PIDTypeDef    gyroyPID;
	PIDTypeDef    gyrozPID;
	PIDTypeDef    Z_posPID;
	PIDTypeDef    Z_ratePID;
	PIDTypeDef    locxPID;
	PIDTypeDef    locyPID;
	PIDTypeDef    locxsPID;
	PIDTypeDef    locysPID;
}CtrlerTypeDef;

typedef struct
{
	float raw_h;
	float f1_h;
	float f2_h;
	float last_h;
	float v_h;
	float f_v_h;
	float f2_v_h;
	float last_high;
}Sensor_Height_TypeDef;

typedef struct
{
	short raw;
	float adjusted;
	float filtered;
	float MetersPerS2;
}DataAcc_TypeDef;

typedef struct
{
	short raw;
	float zero;
	float adjusted;
	float filtered;
	float RadPerSec;
	float DegPerSec;
}DataGyro_TypeDef;

typedef struct
{
   char led_flag;
	 char buzzer_flag;
	 char openmv_flag;
	 char test_flag1;
	 char test_flag2;
	 char test_flag3;
}Sensor_Optical_TypeDef;

typedef struct
{
	short x;
	short y;
	
	int x_i;
	int y_i;
	
	float x_fix;
	float y_fix;
	
	float fixedX;
	float fixedY;
	
	float tempLocx;
	float tempLocy;
	
	unsigned short i_DT;
	unsigned short Ground_Distance;//预留，默认999   0x03E7
	unsigned char valid;// 0:光流不可用         245(0xF5):光流可用
	unsigned char verson;
}Sensor_Optical_LC302_TypeDef;

typedef struct
{
	float MagX_raw;
	float MagY_raw;
	float MagZ_raw;
	float MagX_Adjusted;
	float MagY_Adjusted;
	float MagZ_Adjusted;
}Sensor_Mag_TypeDef;

typedef struct
{
	DataGyro_TypeDef GyroX;
	DataGyro_TypeDef GyroY;
	DataGyro_TypeDef GyroZ;
	DataAcc_TypeDef  AccX;
	DataAcc_TypeDef  AccY;
	DataAcc_TypeDef  AccZ;
}Sensor_6_axis_TypeDef;

typedef struct
{
	Sensor_6_axis_TypeDef          ICM20602Data;
	Sensor_6_axis_TypeDef          MPU6500Data;
	Sensor_Mag_TypeDef             AK8975Data;
	Sensor_Height_TypeDef          SPL06Data;
	Sensor_Height_TypeDef          SonarData;
	Sensor_Height_TypeDef          TofData;
	Sensor_Optical_TypeDef         stm32_flag;
	Sensor_Optical_LC302_TypeDef   OpticalLC302;
}DataTypeDef;

typedef struct
{
	short motor1;
	short motor2;
	short motor3;
	short motor4;
	short motor1_pwm;
	short motor2_pwm;
	short motor3_pwm;
	short motor4_pwm;
}MOTORTypeDef;//1:右上   2：左下    3:左上    4：右下

typedef struct
{
	unsigned short PitCtrler;
	unsigned short RolCtrler;
	unsigned short YawCtrler;
	unsigned short ThrCtrler;
	unsigned short DinggaoSwitch;
	unsigned short DingdianSwitch;
	unsigned short StopSwitch;
}RemoterTypeDef;

typedef struct
{
	unsigned int  PIDingDelayMS;
	unsigned char SensorsStatus;
	unsigned char AdjustStatus;
	unsigned char ARM_Status;
	unsigned char FlyMode;
//	_Bool         Is_gyro_Mode;
//	_Bool         Is_Use_ADRC;
	_Bool         Is_GetingGyroZero;
//	_Bool         Is_SDK_Mode;
//	_Bool         Is_Attitude_Hold_Mode;
//	_Bool         IS_Altitude_Hold_Mode;
//	_Bool         Is_Position_Hold_Mode;
//	_Bool         Is_STOP_Dangerous_Mode;
//	_Bool         Is_Horizontal_Rate_Mode;
}DroneStatusTypeDef;

typedef struct
{
	unsigned int LeftStick_LeftDown_cnt;
	unsigned int LeftStick_LeftUp_cnt;
	unsigned int LeftStick_RightDown_cnt;
	unsigned int LeftStick_RightUp_cnt;
	
	unsigned int RightStick_LeftDown_cnt;
	unsigned int RightStick_LeftUp_cnt;
	unsigned int RightStick_RightDown_cnt;
	unsigned int RightStick_RightUp_cnt;
}StickMotionTypeDef;

typedef struct
{
	unsigned short Uart0_RX_CNT_LastTick;
	unsigned short Uart1_RX_CNT_LastTick;
	unsigned short Uart2_RX_CNT_LastTick;
	unsigned short Uart3_RX_CNT_LastTick;
	unsigned short Uart4_RX_CNT_LastTick;
	unsigned short Uart5_RX_CNT_LastTick;
	unsigned short Uart6_RX_CNT_LastTick;
	unsigned short Uart7_RX_CNT_LastTick;
	
	unsigned short Uart0_RX_CNT_ThisTick;
	unsigned short Uart1_RX_CNT_ThisTick;
	unsigned short Uart2_RX_CNT_ThisTick;
	unsigned short Uart3_RX_CNT_ThisTick;
	unsigned short Uart4_RX_CNT_ThisTick;
	unsigned short Uart5_RX_CNT_ThisTick;
	unsigned short Uart6_RX_CNT_ThisTick;
	unsigned short Uart7_RX_CNT_ThisTick;
	
	unsigned char Is_Uart0_Using_BUFA;
	unsigned char Is_Uart1_Using_BUFA;
	unsigned char Is_Uart2_Using_BUFA;
	unsigned char Is_Uart3_Using_BUFA;
	unsigned char Is_Uart4_Using_BUFA;
	unsigned char Is_Uart5_Using_BUFA;
	unsigned char Is_Uart6_Using_BUFA;
	unsigned char Is_Uart7_Using_BUFA;
}UartManageTypeDef;

extern void*  EventGroupHandler;	//事件标志组句柄

typedef unsigned char  	UCHAR8;/* defined for unsigned 8-bits integer variable 	  无符号8位整型变量  */

typedef struct
{
	float Pitchangle;
	float Yawangle;	
	float rollangle;
	float Gyro_X_Real;
	float Gyro_Y_Real;	
	float Gyro_Z_Real;
	float Acc_X_Real;
	float Acc_Y_Real;
	float Acc_Z_Real;
	char    flag[4];
	float openmv_valid;
	
}ST_SEND_ANGLE;

typedef union
{
	ST_SEND_ANGLE stEnemyE;	
	UCHAR8 ucEData[44];
}UN_AIM_DATA;


typedef struct
{
    struct
    {
        char head[4];             //4
        float Pitchangle;       //4
        float Yawangle;         //4
			  float rollangle;        //4
			  float Gyro_X_Real;        //4
			  float Gyro_Y_Real;        //4
			  float Gyro_Z_Real;        //4
			  char    flag[4];
			  float openmv_valid;
        char tail[2];             //2
    } Send;                     //total:22
} ST_VISION;




#endif

