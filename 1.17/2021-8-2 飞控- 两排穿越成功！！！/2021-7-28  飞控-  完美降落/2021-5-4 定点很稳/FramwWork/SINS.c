#include "SINS.h"
#include "math.h"
#include "SensorsTask.h"
#include "StabilizerTask.h"

#include "DebugerTask.h"

extern UN_AIM_DATA     unAimData;
extern ST_VISION G_ST_Vision ;
//   huifeizhe   Z 3.5 1 3 3        XY  2.5 1 3 3
SINSTypeDef stSINS;

//坐标系旋转
float Sin_Pitch=0,Sin_Roll=0,Sin_Yaw=0;
float Cos_Pitch=0,Cos_Roll=0,Cos_Yaw=0;
float rMat[3][3];
float sqf(float x) {return ((x)*(x));}
void imuComputeRotationMatrix(void)
{
	float Pitch,Roll,Yaw;
	Pitch = roll_imu;
	Roll  = pitch_imu;
	Yaw   = 0;

  Sin_Pitch=sin(Pitch* DEG2RAD);
  Cos_Pitch=cos(Pitch* DEG2RAD);
  Sin_Roll=sin(Roll* DEG2RAD);
  Cos_Roll=cos(Roll* DEG2RAD);
  Sin_Yaw=sin(Yaw* DEG2RAD);
  Cos_Yaw=cos(Yaw* DEG2RAD);
  
  rMat[0][0]=Cos_Yaw* Cos_Roll;
  rMat[0][1]=Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw;
  rMat[0][2]=Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw;
  
  rMat[1][0]=Sin_Yaw * Cos_Roll;
  rMat[1][1]=Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw;
  rMat[1][2]=Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw;
  
  rMat[2][0]=-Sin_Roll;
  rMat[2][1]= Sin_Pitch * Cos_Roll;
  rMat[2][2]= Cos_Pitch * Cos_Roll;
}

void Vector_From_BodyFrame2EarthFrame(Vector3f *bf,Vector3f *ef)
{
  ef->x=rMat[0][0]*bf->x+rMat[0][1]*bf->y+rMat[0][2]*bf->z;
  ef->y=rMat[1][0]*bf->x+rMat[1][1]*bf->y+rMat[1][2]*bf->z;
  ef->z=rMat[2][0]*bf->x+rMat[2][1]*bf->y+rMat[2][2]*bf->z;
}

void Vector_From_EarthFrame2BodyFrame(Vector3f *ef,Vector3f *bf)
{
  bf->x=rMat[0][0]*ef->x+rMat[1][0]*ef->y+rMat[2][0]*ef->z;
  bf->y=rMat[0][1]*ef->x+rMat[1][1]*ef->y+rMat[2][1]*ef->z;
  bf->z=rMat[0][2]*ef->x+rMat[1][2]*ef->y+rMat[2][2]*ef->z;
}

Vector3f Body_Frame,Earth_Frame;
void  SINS_Prepare(void)
{
//  Vector3f Body_Frame,Earth_Frame;

//  Body_Frame.x= data.ICM20602Data.AccX.filtered;
//  Body_Frame.y= data.ICM20602Data.AccY.filtered;
//  Body_Frame.z= data.ICM20602Data.AccZ.filtered;
//	unAimData.stEnemyE.Acc_X_Real=unAimData.stEnemyE.Acc_X_Real*0.01f; //后正
//	unAimData.stEnemyE.Acc_Y_Real=unAimData.stEnemyE.Acc_Y_Real*0.01f;//左正
//	unAimData.stEnemyE.Acc_Z_Real=unAimData.stEnemyE.Acc_Z_Real*0.01f;//左正
	
//  Body_Frame.x= unAimData.stEnemyE.Acc_Y_Real;
//  Body_Frame.y= (unAimData.stEnemyE.Acc_X_Real+0.2f);
//  Body_Frame.z= -unAimData.stEnemyE.Acc_Z_Real;
//	
//	 Body_Frame.x= 0;
//  Body_Frame.y= 0;
//  Body_Frame.z= 0;
	imuComputeRotationMatrix();
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
	
  stSINS.Origin_Acc[_Z]=Earth_Frame.z;
  stSINS.Origin_Acc[_X]=Earth_Frame.x;
  stSINS.Origin_Acc[_Y]=Earth_Frame.y;
  
//  stSINS.Origin_Acc[_Z]*=AcceGravity/AcceMax;
  stSINS.Origin_Acc[_Z]-=AcceGravity;//减去重力加速度
  stSINS.Origin_Acc[_Z]*=100;//加速度cm/s^2
  
//  stSINS.Origin_Acc[_X]*=AcceGravity/AcceMax;
  stSINS.Origin_Acc[_X]*=100;//加速度cm/s^2
  
//  stSINS.Origin_Acc[_Y]*=AcceGravity/AcceMax;
  stSINS.Origin_Acc[_Y]*=100;//加速度cm/s^2
}

/*****************算法技术博客讲解***************************************************
1、四旋翼定高篇之惯导加速度+速度+位置三阶互补融合方案:
http://blog.csdn.net/u011992534/article/details/61924200
2、四旋翼惯导融合之观测传感器滞后问题汇总与巴特沃斯低通滤波器设计
（气压计MS5611、GPS模块M8N、超声波、PX4FLOW等）:
http://blog.csdn.net/u011992534/article/details/73743955
3、从APM源码分析GPS、气压计惯导融合
http://blog.csdn.net/u011992534/article/details/78257684
**********************************************************************************/

/****气压计三阶互补滤波方案——参考开源飞控APM****/
//#define TIME_CONTANST_ZER       1.5f
float TIME_CONTANST_ZER=1.5f;//3.5f;//3.0
#define K_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_POS_ZER               (3.0f / TIME_CONTANST_ZER)
#define SPL06_Sync_Cnt 40//20
uint16 High_Delay_Cnt = SPL06_Sync_Cnt;


void Strapdown_INS_High(float height_INS_raw)//  cm  !!!!!!!!!!!!!
{
  uint16 Cnt=0;
	float Altitude_Delta=0;
  static uint16_t Save_Cnt=0;
  Save_Cnt++;//数据存储周期
  
	SINS_Prepare();
	
	//height_INS_raw  //高度观测量  HC_SR04_Distance*Cos_Roll*Cos_Pitch;

  //由观测量得到状态误差　　//测量值与惯导估计量的差，单位cm
  Altitude_Delta=height_INS_raw-stSINS.Pos_History[_Z][High_Delay_Cnt];
  
	//三路积分反馈量修正惯导
  stSINS.acc_correction[_Z] += Altitude_Delta* K_ACC_ZER*H_DT ;//加速度矫正量
  stSINS.vel_correction[_Z] += Altitude_Delta* K_VEL_ZER*H_DT ;//速度矫正量
  stSINS.pos_correction[_Z] += Altitude_Delta* K_POS_ZER*H_DT ;//位置矫正量
	
  //加速度计矫正后更新
  stSINS.Last_Acc[_Z]=stSINS.Acc[_Z];//上一次加速度量
  stSINS.Acc[_Z]=stSINS.Origin_Acc[_Z]+stSINS.acc_correction[_Z];
  //速度增量矫正后更新，用于更新位置,由于步长h=0.005,相对较长，
  //这里采用二阶龙格库塔法更新微分方程，不建议用更高阶段，因为加速度信号非平滑
  stSINS.SpeedDelta[_Z]=(stSINS.Last_Acc[_Z]
                    +stSINS.Acc[_Z])*H_DT/2.0f;
  //原始位置更新
  stSINS.Origin_Pos[_Z]+=(stSINS.Speed[_Z]+0.5f*stSINS.SpeedDelta[_Z])*H_DT;
  //位置矫正后更新
  stSINS.Position[_Z]=stSINS.Origin_Pos[_Z]+stSINS.pos_correction[_Z];
  //原始速度更新
  stSINS.Origin_Vel[_Z]+=stSINS.SpeedDelta[_Z];
  //速度矫正后更新
  stSINS.Speed[_Z]=stSINS.Origin_Vel[_Z]+stSINS.vel_correction[_Z];
  
  if(Save_Cnt>=1)// 1*5ms
  {
    for(Cnt=Save_Num-1;Cnt>0;Cnt--)//滑动一次
    {
      stSINS.Pos_History[_Z][Cnt]=stSINS.Pos_History[_Z][Cnt-1];
    }
    stSINS.Pos_History[_Z][0]=stSINS.Position[_Z];
    Save_Cnt=0;
  }
}


float TIME_CONTANST_XY =   0.1f;
#define K_ACC_XY	     (1.0f / (TIME_CONTANST_XY * TIME_CONTANST_XY * TIME_CONTANST_XY))
#define K_VEL_XY             (9.0f / (TIME_CONTANST_XY * TIME_CONTANST_XY))
#define K_POS_XY             (7.0f / TIME_CONTANST_XY)
//   0.1  1   9  7             0
//float TIME_CONTANST_XY =   2.5f;
//float K_ACC_XY;	    
//float K_VEL_XY ;        
//float K_POS_XY  ;     
//float k1=1.0f,k2=3.0f,k3=3.0f;

uint16_t LocXY_SINS_Delay_Cnt=0;//100ms

void Strapdown_INS_Horizontal(float locx,float locy)
{
  uint16 Cnt=0;
	float locX_Delta=0,locY_Delta=0;
	static uint16_t LocXY_Save_Period_Cnt=0;
	
	
	
//	K_ACC_XY=    (k1 / (TIME_CONTANST_XY * TIME_CONTANST_XY * TIME_CONTANST_XY));
//	K_VEL_XY =    (k2 / (TIME_CONTANST_XY * TIME_CONTANST_XY));
//	K_POS_XY  =     (k3 / TIME_CONTANST_XY);
	
	
	
	
  LocXY_Save_Period_Cnt++;
  if(LocXY_Save_Period_Cnt>=2)//10ms
  {
    for(Cnt=Save_Num-1;Cnt>0;Cnt--)//10ms滑动一次
    {
      stSINS.Pos_History[_X][Cnt]=stSINS.Pos_History[_X][Cnt-1];
      stSINS.Pos_History[_Y][Cnt]=stSINS.Pos_History[_Y][Cnt-1];
    }
    stSINS.Pos_History[_X][0]=stSINS.Position[_X];
    stSINS.Pos_History[_Y][0]=stSINS.Position[_Y];
    LocXY_Save_Period_Cnt=0;
  }
	//先保存还是先计算？？只差5ms，应该影响不大   主要是保证保存的不是0就行吧。。
	
	//optical   x+ 2-  2=Y         y+   1-  1=X
  //    GPS/光流位置 与 惯导估计量的差，单位cm 加延时修正
  locX_Delta = -locy - stSINS.Pos_History[_X][LocXY_SINS_Delay_Cnt];
  locY_Delta = -locx - stSINS.Pos_History[_Y][LocXY_SINS_Delay_Cnt];
  


  stSINS.acc_correction[_X] += locX_Delta* K_ACC_XY*LocXY_DT;//加速度矫正量
  stSINS.vel_correction[_X] += locX_Delta* K_VEL_XY*LocXY_DT;//速度矫正量
  stSINS.pos_correction[_X] += locX_Delta* K_POS_XY*LocXY_DT;//位置矫正量
  
  stSINS.acc_correction[_Y] += locY_Delta* K_ACC_XY*LocXY_DT;//加速度矫正量
  stSINS.vel_correction[_Y] += locY_Delta* K_VEL_XY*LocXY_DT;//速度矫正量
  stSINS.pos_correction[_Y] += locY_Delta* K_POS_XY*LocXY_DT;//位置矫正量
  
	
//	stSINS.Origin_Acc[_X] = stSINS.Origin_Acc[_Y] = 0;
  /********************  X  *****************************************/
  //水平运动加速度计校正
  stSINS.Acc[_X]=stSINS.Origin_Acc[_X]+stSINS.acc_correction[_X];
  //速度增量矫正后更新，用于更新位置
  stSINS.SpeedDelta[_X]=stSINS.Acc[_X]*LocXY_DT;
  //原始位置更新
  stSINS.Origin_Pos[_X]+=(stSINS.Speed[_X]+0.5f*stSINS.SpeedDelta[_X])*LocXY_DT;
  //位置矫正后更新
  stSINS.Position[_X]=stSINS.Origin_Pos[_X]+stSINS.pos_correction[_X];
  //原始速度更新
  stSINS.Origin_Vel[_X]+=stSINS.SpeedDelta[_X];
  //速度矫正后更新
  stSINS.Speed[_X]=stSINS.Origin_Vel[_X]+stSINS.vel_correction[_X];
  
  /********************  Y　*****************************************/
  //水平运动加速度计校正
  stSINS.Acc[_Y]=stSINS.Origin_Acc[_Y]+stSINS.acc_correction[_Y];
  //速度增量矫正后更新，用于更新位置
  stSINS.SpeedDelta[_Y]=stSINS.Acc[_Y]*LocXY_DT;
  //原始位置更新
  stSINS.Origin_Pos[_Y]+=(stSINS.Speed[_Y]+0.5f*stSINS.SpeedDelta[_Y])*LocXY_DT;
  //位置矫正后更新
  stSINS.Position[_Y]=stSINS.Origin_Pos[_Y]+stSINS.pos_correction[_Y];
  //原始速度更新
  stSINS.Origin_Vel[_Y]+=stSINS.SpeedDelta[_Y];
  //速度矫正后更新
  stSINS.Speed[_Y]=stSINS.Origin_Vel[_Y]+stSINS.vel_correction[_Y];
}
