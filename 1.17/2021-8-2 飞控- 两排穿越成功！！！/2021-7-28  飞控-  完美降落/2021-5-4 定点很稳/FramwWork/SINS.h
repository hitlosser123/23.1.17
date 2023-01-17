#ifndef __SINS_H
#define __SINS_H
#include "includes.h"

typedef   signed  char   int8;
typedef unsigned  char   uint8;
typedef unsigned  char   byte;
typedef   signed  short  int16;
typedef unsigned  short  uint16;

#define LocXY_DT  0.005f
#define   H_DT    0.005f
#define _Z    0
#define _X    1
#define _Y    2

#define MAX(a,b)  ((a)>(b)?(a):(b))
#define MIN(a,b)  ((a)<(b)?(a):(b))



typedef struct
{
  float x;
  float y;
  float z;
}Vector3f;



#define Axis_Num  3
#define Save_Num  50
typedef struct
{
	float Position[Axis_Num];//位置估计量
	float Speed[Axis_Num];//速度估计量
	float Acc[Axis_Num];//加速度估计量
	float Pos_History[Axis_Num][Save_Num];//历史惯导位置
	float Last_Acc[Axis_Num];
	float Origin_Pos[Axis_Num];
	float Origin_Vel[Axis_Num];
	float Origin_Acc[Axis_Num];
	float SpeedDelta[Axis_Num];
	
	float pos_correction[Axis_Num];
	float acc_correction[Axis_Num];
	float vel_correction[Axis_Num];
}SINSTypeDef;//   Strapdown inertial navigation system


#define AcceMax     4096  //   4096
#define AcceGravity 9.80f


#ifndef M_PI_F
 #define M_PI_F 3.141592653589793f
#endif
//#ifndef PI
// # define PI M_PI_F
//#endif
#ifndef M_PI_2
 # define M_PI_2 1.570796326794897f
#endif
//Single precision conversions
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f


extern SINSTypeDef   stSINS;
extern float Sin_Pitch,Sin_Roll,Sin_Yaw;
extern float Cos_Pitch,Cos_Roll,Cos_Yaw;

float constrain_float(float amt, float low, float high);
int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high);

void  SINS_Prepare(void);
void  Strapdown_INS_High(float height_INS_raw);//  cm  !!!!!!!!!!!!!
void Strapdown_INS_Horizontal(float locx,float locy);


#endif
