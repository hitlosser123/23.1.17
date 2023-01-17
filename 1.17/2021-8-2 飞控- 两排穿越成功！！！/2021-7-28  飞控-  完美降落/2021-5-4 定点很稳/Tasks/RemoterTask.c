#include "RemoterTask.h"
#include "pid.h"
#include "AutoflyTask.h"
#include "uart2_dma.h"

float channel[4];
RemoterTypeDef  Remoter;
DroneStatusTypeDef DroneStatus;
StickMotionTypeDef StickMotion;

//  �����»���4��ͨ��1024����ʧ�ر���
//  2019-8-9  6208���ջ�ʧ������
//  ������������ʧ�ܣ��ж�ҡ����λ���������жϵ���SBUSԭʼ���� 240  1807
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
				sbus_channel[3] = 1024;//��ʱ��ôд
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
//			else//��ʱ
//			{
//				DroneStatus.ARM_Status = DisArmed;//����14SGʧ�ر������ܣ�����������������
//				Remoter.PitCtrler = 3000;
//				Remoter.RolCtrler = 3000;
//				Remoter.ThrCtrler = 2000;
//				Remoter.YawCtrler = 3000; 
//			}

			vTaskDelay(10);//�¼���־�黹û�У�����ʱ10ms��Ҳ����10��ʱ�ӽ���
			
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
//								EventGroupHandler,	//�¼���־����
//								got_SBUS_BIT,
//								pdTRUE,        /* BITs should be cleared before returning. */
//								pdTRUE,       /* �ȴ����б�־λ */
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
//			else//��ʱ
//			{
//				DroneStatus.ARM_Status = DisArmed;//����14SGʧ�ر������ܣ�����������������
//				Remoter.PitCtrler = 3000;
//				Remoter.RolCtrler = 3000;
//				Remoter.ThrCtrler = 2000;
//				Remoter.YawCtrler = 3000; 
//			}
//		}
//		else
//		{
//			vTaskDelay(10);//�¼���־�黹û�У�����ʱ10ms��Ҳ����10��ʱ�ӽ���
//		}
//	}
//}


#define is_Stick_MAX(value)      ( value>3900 &&  value<4100)//4000
#define is_Stick_MIN(value)      ( value>1900 &&  value<2100)//2000
#define is_Stick_MID(value)      ( value>2900 &&  value<3100)//3000
//2018/7/3�޸Ľ���������ֵ,ʱ�䣬�޸��߼�������λ���������
//2019-6-27 ��������ҡ�˶���
void Check_Stick_Motion(void)
{
	if( is_Stick_MIN(Remoter.ThrCtrler) &&  is_Stick_MAX(Remoter.YawCtrler) )//  arm  ����
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
		DroneStatus.ARM_Status=Armed;//���½���

		if( StickMotion.RightStick_LeftDown_cnt!=0)  //�Ҳ�  �����½���SDKģʽ
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
		DroneStatus.ARM_Status=DisArmed;//��������
		StickMotion.LeftStick_RightDown_cnt=0;StickMotion.LeftStick_LeftDown_cnt=0;
	}
	
	
//		if(KeySDKflag)    //��������ȼ���Ȼ��ô�ߣ�����������
//	{
//		DroneStatus.FlyMode = FlyMode_SDK;
//		DroneStatus.ARM_Status = Armed;
//	}
	
}


#define      _Height_Hold_sw_channel    9//9    ����  1696            352normal
#define      _Pos_Hold_sw_channel       8//8    352 Ĭ��      1024  ˮƽ�ٶȻ�        1696  ˮƽλ�û�
#define      _Stop_Dangerous_sw_channel 5//7    352 Ĭ��           ������ͣ��
#define      _Attitude_Hold_sw_channel  4//4    352 ����              1024 ���ٶȻ�       1696 yaw
#define      _ADRC_sw_channel  					7//5    352 PID          1024 ���ٶȻ� ADRC      1696 ����
#define      _14SG_LD_channel           10
#define      _14SG_RD_channel           11
#define      _14SG_LS_channel           4
#define      _14SG_RS_channel           7

/******************
����ģʽ������
���٣�����ģʽ���е���,      ���ߡ����㡢ˮƽ�ٶȻ���SDKģʽ�������е��٣�������Ͷ�Ӧ����½��ٶ�
ҡ��ʹ�ܣ�SDKģʽ������ҡ�ˣ�����ҡ�������������Կ����ٶȻ�
����ͣ����ʼ����Ч��ģʽ�л���PWM���ʱ������жϣ�����ſɷ��У�ע��PWM���ֻ�ܽ�һ��if
��ģʽ���⣺���߼�������ģʽ������ģʽ�������������ݣ��������ڶ��߻���֮�ϣ�SDKģʽ�ڶ������֮��
������״̬�����㡢SDK�����߿�����Ҫ�ж���Ӧ������״̬
*********************/
void Check_Fly_Mode(void)
{

	Check_Stick_Motion();
	  
		if( sbus_channel[4] <=500 )
	{		
		DroneStatus.FlyMode = FlyMode_DangerousStop;//����ͣ��
	}
	else
	{
     DroneStatus.FlyMode = FlyMode_SDK ;
	}
}

