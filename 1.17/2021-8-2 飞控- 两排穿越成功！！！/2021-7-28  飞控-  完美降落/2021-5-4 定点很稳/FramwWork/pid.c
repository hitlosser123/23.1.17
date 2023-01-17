#include "pid.h"
#include "adrc.h"
#include "RemoterTask.h"
#include "StabilizerTask.h"

//   65535/1000=65.535f      PID<65.535f
//  V_h  KP    100x to pid table           ctrler 384  = pid table
CtrlerTypeDef Ctrler={

/*des FB  Kp   Ki    Kd Up Ui Ud E preE SumE U Umax Upmax Uimax Udmax SumEmax Emin***/
	{ 0,  0, 3.0, 0.1,  8, 0, 0, 0, 0, 0,   0,  0, 200,  200,  0,  10,  1000,     3},//pitch//p=15
	{ 0,  0, 3.0, 0.1,  8, 0, 0, 0, 0, 0,   0,  0, 200,  200,  0,  10,  1000,     3},//roll
	{ 0,  0, 4.0, 0.04,  0, 0, 0, 0, 0, 0,   0,  0, 60,  60,  2,  0,     50,     2},//yaw
	//skywalker  4         letian40A    3       pitch/roll KP
	//yaw KP   3   -> 4
	
	//skywalker  6    letian40A    5           gyrox,y   KP
	//skywalker  15    letian40A   10           gyrox,y   KD
	{ 0,  0, 5   , 0.01 ,10, 0,0, 0, 0, 0,   0,  0, 300,  300,  20,  100,  1000,   2},//gyrox
	{ 0,  0, 5   , 0.01 ,10, 0,0, 0, 0, 0,   0,  0, 300,  300,  20,  100,  1000,   2},//gyroy
	{ 0,  0, 6.0, 0.001 ,0.02, 0,0, 0, 0, 0,  0,  0, 250,  250,  60,  10,   2000,     20},//gyroz
	
	
//	{ 0,  0, 1,    0.02 ,0.06, 0.1,0, 0, 0, 0,   0,  0, 1.0 , 0.9,  0.3,  0.3,  30,     0.1},//h
//	{ 0,  0, 500,  0.5,  0,0,0, 0, 0, 0,   0, 0,300, 300, 60,   60,    30,    0.1},//h rate
	
	{ 0,  0, 0.7, 0.005 ,0.1, 0,0, 0, 0, 0,   0,  0, 1.0 , 0.9,  0.3,  0.3,  30,     0.3},//h
	{ 0,  0, 400,  0.435,  0,0,0, 0, 0, 0,   0, 0,300, 300, 60,   60,    30,    0.1},//h rate
	
//  { 0,  0, 0.7, 0.01 ,0.067, 0,0, 0, 0, 0,   0,  0, 1.0 , 0.9,  0.3,  0.3,  30,     0.1},//h
//	{ 0,  0, 384,  0.435,  0,0,0, 0, 0, 0,   0, 0,300, 300, 60,   60,    30,    0.1},//h rate
  //384,  0.435,  0     letian40A  vz
	//600.0, 0.935,13.4   skywalker20A   vz
	
	/*des FB  Kp   Ki  Kd   Up Ui Ud E preE SumE U Umax Upmax Uimax Udmax SumEmax Emin***/	
	//hui fei zhe   1.8outKP    1.8 0.45inKP KI
	{ 0,  0, 0.5    , 0.01 ,0.00, 0,0, 0, 0, 0,   0,  0, 150, 150,  20,   50,     200,     30},//locx
	{ 0,  0, 0.5    , 0.01 ,0.00, 0,0, 0, 0, 0,   0,  0, 150, 150,  20,   50,     200,     30},//locy
	{ 0,  0, 5      , 0 ,2.00, 0,0, 0, 0, 0,   0,  0, 500, 450, 100,   100,     200,   10},//locxs
	{ 0,  0, 5      , 0 ,2.00, 0,0, 0, 0, 0,   0,  0, 500, 450, 100,   100,     200,   10},//locys

};           //yaw  kp  6
//8->6//uimax 20


/******积分分离、积分限幅、P I D 输出限幅、总输出限幅***********************/
void ComputePID(PIDTypeDef *pPID)
{
	pPID->E = pPID->Des - pPID->FB;//计算当前偏差

	if(((pPID->U <= pPID->UMax && pPID->E > 0) || (pPID->U >= -pPID->UMax && pPID->E < 0)) \
		    && abs(pPID->E) < pPID->EMin)//积分分离
	{
		pPID->SumE += pPID->E;//计算偏差积分
	}
	value_limit( pPID->SumE , -pPID->SumEMax , pPID->SumEMax );//积分限幅
	pPID->Ui = pPID->Ki * pPID->SumE;
	value_limit( pPID->Ui , -pPID->UiMax , pPID->UiMax );
	
	pPID->Up = pPID->Kp * pPID->E;
	value_limit( pPID->Up , -pPID->UpMax , pPID->UpMax );
	
	pPID->Ud = pPID->Kd * ( pPID->E - pPID->PreE );
	value_limit( pPID->Ud , -pPID->UdMax , pPID->UdMax );
	
	pPID->U = pPID->Up + pPID->Ui + pPID->Ud;/*位置式PID计算公式*/
  value_limit( pPID->U , -pPID->UMax , pPID->UMax );  /*PID运算输出限幅*/	
	
	pPID->PreE = pPID->E ;//保存本次偏差
}

float alpha= 0.75;
float zeta = 2.0 ;//0.75   1.7
/******Kp非线性*********************/
void ComputeNLPID(PIDTypeDef *pPID)
{
	pPID->E = pPID->Des - pPID->FB;//计算当前偏差

	if(((pPID->U <= pPID->UMax && pPID->E > 0) || (pPID->U >= -pPID->UMax && pPID->E < 0)) \
		    && abs(pPID->E) < pPID->EMin)//积分分离
	{
		pPID->SumE += pPID->E;//计算偏差积分
	}
	value_limit( pPID->SumE , -pPID->SumEMax , pPID->SumEMax );//积分限幅
	pPID->Ui = pPID->Ki * pPID->SumE;
	value_limit( pPID->Ui , -pPID->UiMax , pPID->UiMax );
	
	pPID->Up = pPID->Kp * Fal_ADRC(pPID->E,alpha,zeta);
	value_limit( pPID->Up , -pPID->UpMax , pPID->UpMax );
	
	pPID->Ud = pPID->Kd * ( pPID->E - pPID->PreE );
	value_limit( pPID->Ud , -pPID->UdMax , pPID->UdMax );
	
	pPID->U = pPID->Up + pPID->Ui + pPID->Ud;/*位置式PID计算公式*/
  value_limit( pPID->U , -pPID->UMax , pPID->UMax );  /*PID运算输出限幅*/	
	
	pPID->PreE = pPID->E ;//保存本次偏差
}

//Fal_ADRC(p,0.8,3);

void ComputeYawPID(PIDTypeDef *pPID)
{
	pPID->E = pPID->Des - pPID->FB;//计算当前偏差
	
	if(pPID->E>=180)pPID->E-=360;
	if(pPID->E<=-180)pPID->E+=360;

	if(((pPID->U <= pPID->UMax && pPID->E > 0) || (pPID->U >= -pPID->UMax && pPID->E < 0)) \
		    && abs(pPID->E) < pPID->EMin)//积分分离
	{
		pPID->SumE += pPID->E;//计算偏差积分
	}
	value_limit( pPID->SumE , -pPID->SumEMax , pPID->SumEMax );//积分限幅
	pPID->Ui = pPID->Ki * pPID->SumE;
	value_limit( pPID->Ui , -pPID->UiMax , pPID->UiMax );
	
	pPID->Up = pPID->Kp * pPID->E;
	value_limit( pPID->Up , -pPID->UpMax , pPID->UpMax );
	
	pPID->Ud = pPID->Kd * ( pPID->E - pPID->PreE );
	value_limit( pPID->Ud , -pPID->UdMax , pPID->UdMax );
	
	pPID->U = pPID->Up + pPID->Ui + pPID->Ud;/*位置式PID计算公式*/
  value_limit( pPID->U , -pPID->UMax , pPID->UMax );  /*PID运算输出限幅*/	
	
	pPID->PreE = pPID->E ;//保存本次偏差
}



void Clear_Structure(void)
{
	Ctrler.pitchPID.SumE=0;
	Ctrler.rollPID.SumE=0;
	Ctrler.yawPID.SumE=0;
	Ctrler.gyroxPID.SumE=0;
	Ctrler.gyroyPID.SumE=0;
	Ctrler.gyrozPID.SumE=0;
	Ctrler.Z_posPID.SumE=0;
	Ctrler.Z_ratePID.SumE=0;

//	DroneStatus.FlyMode = FlyMode_AttitudeHold ;
}
