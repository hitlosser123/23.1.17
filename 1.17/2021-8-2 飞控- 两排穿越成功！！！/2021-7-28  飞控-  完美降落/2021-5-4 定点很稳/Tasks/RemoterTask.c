#include "RemoterTask.h"
#include "pid.h"
#include "AutoflyTask.h"
#include "uart2_dma.h"

float channel[4];
RemoterTypeDef  Remoter;
DroneStatusTypeDef DroneStatus;
StickMotionTypeDef StickMotion;

//  不更新或者4个通道1024就是失控保护
//  2019-8-9  6208接收机失控问题
//  按键启动定高失败：判断摇杆中位方法错误，判断的是SBUS原始数据 240  1807
unsigned char FailSafeFlag=0;
void remoter_task(void *pvParameters)
{
	static unsigned int ticks=0;
	static unsigned int last_SBUS_CNT;
	while(1)
	{
		if(ticks%20==0)
		{
			if(SBUS_ValidCnt == last_SBUS_CNT )
			{
				FailSafeFlag = 1;
				
				sbus_channel[0] = 1024;
				sbus_channel[1] = 1024;
				sbus_channel[2] = 1024;
				sbus_channel[3] = 1024;//暂时这么写
			}
			else FailSafeFlag = 0;
			last_SBUS_CNT = SBUS_ValidCnt;
		}
		
		if(  sbus_channel[0] == 1024 &&   sbus_channel[1] == 1024  && \
			   sbus_channel[2] == 1024  &&  sbus_channel[3] == 1024  )
		{
			FailSafeFlag = 1;
		}
				//1696 352  672
				channel[0]= (sbus_channel[0]-1024)/672.0*1000.0+3000.0; 
				channel[1]= (sbus_channel[1]-1024)/672.0*1000.0+3000.0; 
				channel[2]= (sbus_channel[2]-1024)/672.0*1000.0+3000.0; 
				channel[3]= (sbus_channel[3]-1024)/672.0*1000.0+3000.0; 
				
				value_limit(channel[2],2000,4000);
				if(channel[0]<1800||channel[0]>4200)channel[0]=3000;
				if(channel[1]<1800||channel[1]>4200)channel[1]=3000;
				if(channel[3]<1800||channel[3]>4200)channel[3]=3000;
				
				Remoter.PitCtrler	= channel[1] ;
				Remoter.RolCtrler = channel[0] ;
				Remoter.ThrCtrler = channel[2] ;
				Remoter.YawCtrler = channel[3] ;		
		
		
				if(  FailSafeFlag == 1 &&  KeySDKflag == 1  )
				{
					Remoter.PitCtrler	= 3000 ;
					Remoter.RolCtrler = 3000 ;
					Remoter.ThrCtrler = 3000 ;
					Remoter.YawCtrler = 3000 ;						
				}
//			else//超时
//			{
//				DroneStatus.ARM_Status = DisArmed;//设置14SG失控保护功能！！！！！！！！！
//				Remoter.PitCtrler = 3000;
//				Remoter.RolCtrler = 3000;
//				Remoter.ThrCtrler = 2000;
//				Remoter.YawCtrler = 3000; 
//			}

			vTaskDelay(10);//事件标志组还没有，就延时10ms，也就是10个时钟节拍
			
			ticks++;
			
			
	}
}

//void remoter_task(void *pvParameters)
//{
//	EventBits_t uxBits;
//	const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
//	while(1)
//	{
//		if(EventGroupHandler!=NULL)
//		{
//			uxBits = xEventGroupWaitBits(
//								EventGroupHandler,	//事件标志组句柄
//								got_SBUS_BIT,
//								pdTRUE,        /* BITs should be cleared before returning. */
//								pdTRUE,       /* 等待所有标志位 */
//								xTicksToWait );/* Wait a maximum of 100ms for either bit to be set. */
//			if( ( uxBits & (got_SBUS_BIT) ) != 0 )
//			{
//				
//				channel[0]= (sbus_channel[0]-1024)/690.0*1000.0+3000.0; 
//				channel[1]= (sbus_channel[1]-1024)/690.0*1000.0+3000.0; 
//				channel[2]= (sbus_channel[2]-1024)/690.0*1000.0+3000.0; 
//				channel[3]= (sbus_channel[3]-1024)/690.0*1000.0+3000.0; 
//				
//				value_limit(channel[2],2000,4000);
//				if(channel[0]<1800||channel[0]>4200)channel[0]=3000;
//				if(channel[1]<1800||channel[1]>4200)channel[1]=3000;
//				if(channel[3]<1800||channel[3]>4200)channel[3]=3000;
//				
//				Remoter.PitCtrler	= channel[1] ;
//				Remoter.RolCtrler = channel[0] ;
//				Remoter.ThrCtrler = channel[2] ;
//				Remoter.YawCtrler = channel[3] ;		
//			}
//			else//超时
//			{
//				DroneStatus.ARM_Status = DisArmed;//设置14SG失控保护功能！！！！！！！！！
//				Remoter.PitCtrler = 3000;
//				Remoter.RolCtrler = 3000;
//				Remoter.ThrCtrler = 2000;
//				Remoter.YawCtrler = 3000; 
//			}
//		}
//		else
//		{
//			vTaskDelay(10);//事件标志组还没有，就延时10ms，也就是10个时钟节拍
//		}
//	}
//}


#define is_Stick_MAX(value)      ( value>3900 &&  value<4100)//4000
#define is_Stick_MIN(value)      ( value>1900 &&  value<2100)//2000
#define is_Stick_MID(value)      ( value>2900 &&  value<3100)//3000
//2018/7/3修改解锁上锁阈值,时间，修改逻辑，不在位置清零计数
//2019-6-27 加入其他摇杆动作
void Check_Stick_Motion(void)
{
	if( is_Stick_MIN(Remoter.ThrCtrler) &&  is_Stick_MAX(Remoter.YawCtrler) )//  arm  右下
		StickMotion.LeftStick_RightDown_cnt++;
	else StickMotion.LeftStick_RightDown_cnt=0;
	
	if( is_Stick_MIN(Remoter.ThrCtrler) &&  is_Stick_MIN(Remoter.YawCtrler) )  // disarm
		StickMotion.LeftStick_LeftDown_cnt++;
	else StickMotion.LeftStick_LeftDown_cnt=0;
	
	if( is_Stick_MAX(Remoter.ThrCtrler) &&  is_Stick_MIN(Remoter.YawCtrler) )  // adjust
		StickMotion.LeftStick_LeftUp_cnt++;
	else StickMotion.LeftStick_LeftUp_cnt=0;

	if( is_Stick_MAX(Remoter.ThrCtrler) &&  is_Stick_MAX(Remoter.YawCtrler) )  // debug
		StickMotion.LeftStick_RightUp_cnt++;
	else StickMotion.LeftStick_RightUp_cnt=0;
	
	
	if( is_Stick_MIN(Remoter.PitCtrler) &&  is_Stick_MIN(Remoter.RolCtrler) )  // 
		StickMotion.RightStick_LeftDown_cnt++;
	else StickMotion.RightStick_LeftDown_cnt=0;
	
	if( is_Stick_MIN(Remoter.PitCtrler) &&  is_Stick_MAX(Remoter.RolCtrler) )  // 
		StickMotion.RightStick_RightDown_cnt++;
	else StickMotion.RightStick_RightDown_cnt=0;

	if( is_Stick_MAX(Remoter.PitCtrler) &&  is_Stick_MIN(Remoter.RolCtrler) )  // 
		StickMotion.RightStick_LeftUp_cnt++;
	else StickMotion.RightStick_LeftUp_cnt=0;

	if( is_Stick_MAX(Remoter.PitCtrler) &&  is_Stick_MAX(Remoter.RolCtrler) )  // 
		StickMotion.RightStick_RightUp_cnt++;
	else StickMotion.RightStick_RightUp_cnt=0;

	
	
	if(StickMotion.LeftStick_RightDown_cnt>=ARM_Delay_time)
	{
		DroneStatus.ARM_Status=Armed;//右下解锁

		if( StickMotion.RightStick_LeftDown_cnt!=0)  //右侧  的左下进入SDK模式
		{
			DroneStatus.FlyMode = FlyMode_SDK;
		}
			
		else
		{
		}
		StickMotion.LeftStick_RightDown_cnt=0;StickMotion.LeftStick_LeftDown_cnt=0;
	}
	if(StickMotion.LeftStick_LeftDown_cnt>=DISARM_Delay_time)
	{
		DroneStatus.ARM_Status=DisArmed;//左下上锁
		StickMotion.LeftStick_RightDown_cnt=0;StickMotion.LeftStick_LeftDown_cnt=0;
	}
	
	
//		if(KeySDKflag)    //这个的优先级居然这么高？？？？？？
//	{
//		DroneStatus.FlyMode = FlyMode_SDK;
//		DroneStatus.ARM_Status = Armed;
//	}
	
}


#define      _Height_Hold_sw_channel    9//9    定高  1696            352normal
#define      _Pos_Hold_sw_channel       8//8    352 默认      1024  水平速度环        1696  水平位置环
#define      _Stop_Dangerous_sw_channel 5//7    352 默认           其它：停机
#define      _Attitude_Hold_sw_channel  4//4    352 自稳              1024 角速度环       1696 yaw
#define      _ADRC_sw_channel  					7//5    352 PID          1024 角速度环 ADRC      1696 串级
#define      _14SG_LD_channel           10
#define      _14SG_RD_channel           11
#define      _14SG_LS_channel           4
#define      _14SG_RS_channel           7

/******************
飞行模式分析：
怠速：自稳模式，有怠速,      定高、定点、水平速度环、SDK模式，不能有怠速，油门最低对应最大下降速度
摇杆使能：SDK模式不禁用摇杆，增大摇杆死区，但可以控制速度环
紧急停机：始终有效，模式切换、PWM输出时，最后判断，满足才可飞行，注意PWM输出只能进一个if
各模式互斥：定高兼容其他模式，自稳模式与其他都不兼容，定点需在定高基础之上，SDK模式在定点基础之上
传感器状态：定点、SDK、定高开启需要判断相应传感器状态
*********************/
void Check_Fly_Mode(void)
{

	Check_Stick_Motion();
	  
		if( sbus_channel[4] <=500 )
	{		
		DroneStatus.FlyMode = FlyMode_DangerousStop;//紧急停机
	}
	else
	{
     DroneStatus.FlyMode = FlyMode_SDK ;
	}
}

