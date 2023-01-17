#include "AutoflyTask.h"
#include "delay.h"
#include "RemoterTask.h"
#include "pid.h"
#include "openmv.h"
#include "led.h"
#include "StabilizerTask.h"
#include "config&param.h"   

//    Red, 1
//    Orange,2
//    Yellow,3
//    Green,4
//    Blue,0
//    Cyan,6
//    Purple,7
//    White,7
//    Black,9

float x_test = 1;
float y_test = 1;
float yaw_test = 0;
float ForwardDist=380.0f,LeftDist=115.0f;//380   115

unsigned int SDK_StateMachine[200];
unsigned int CurrentSDKState,SDKStateMAX;

unsigned int landcnt=0;
unsigned int landcnt1=0;
unsigned int searchcnt=0;
unsigned int searchcnt1=0;
unsigned int yawcnt=0;
unsigned int yawcntsearch=0;
float   last_search_yaw = 0;
float   yaw_Des = 0;
float   yaw_Des1 = 0;
float   yaw_Des2 = 0;
float   yaw_init = 0;
float   yaw_SDK = 0;
float   yaw_SDK_openmv[100];
float   time_SDK_cnt= 0 ;
unsigned int   yaw_cnt = 0;  //记录杆的数量
unsigned int KeyPressedTimeMS;
unsigned char KeySDKflag=0,SDK_StateChangeFlag=0,SDK_DelayWakeFlag=0,ShotQRCodeFlag=0,SDKLandFlag=0;
float temp_V_x=-1,temp_V_y=-1,temp_Gyroz=0,temp_V_h=0,temp_Yaw,temp_LocX,temp_LocY,temp_V_x2021=0,temp_V_y2021=0;
void AutoflyTask(void *pvParameters)
{
 
	static u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt(); 
	SDK_StateMachine_Init();
//  OpenMVData.stm32_flag.led_flag =1; 
	OpenMVData.stm32_flag.openmv_flag=0;  //openmv标志位
	while(1)
	{

		vTaskDelayUntil(&lastWakeTime, 5 );		/*控制周期延时*/ 
//		if(OpenMVData.openmv_top.key==1 )
//		{

		if(  sbus_channel[5] <=500 &&sbus_channel[4] >=1500 )KeyPressedTimeMS+=5;
		else KeyPressedTimeMS=0;
//		}
		
		if(KeyPressedTimeMS >=2000)
		{
			KeySDKflag =1;
			DroneStatus.ARM_Status = Armed;
			OpenMVData.stm32_flag.led_flag =5; 
		}
		else 
		{
			KeySDKflag =0;
			data.TofData.last_high = 0;
		}
		
		if(KeySDKflag ==1)
		{
			SDK_StateMachine_Loop(); 
		}
		else
		{
			SDK_StateMachine_Reset();
		}
		tick++;		
	}
}

#define SDK_Cmd_TakeOff        0  //起飞
#define SDK_Cmd_Land           1  //降落
#define SDK_Cmd_Search0        2  //原地转圈搜索
#define SDK_Cmd_Search1        3  //矩形移动搜索
#define SDK_Cmd_PosHold        4  //定点
#define SDK_Cmd_Circle         5  //起飞点的圆
#define SDK_Cmd_FollowLine     6  //循线
#define SDK_Cmd_PowerLine      7  //循电线    电赛专用
#define SDK_Cmd_Surround       8  //绕飞      电赛专用
#define SDK_Cmd_GetLine        9  //进线    电赛专用
#define SDK_Cmd_GetClose       10 //靠近    电赛专用
#define SDK_Cmd_DelayWake      11  //延时启动
#define SDK_Cmd_Pos1           12  //开环1
#define SDK_Cmd_Pos2           13  //开环2
#define SDK_Cmd_Pos3           14  //开环3   
#define SDK_Cmd_Pos4           15  //开环4
#define SDK_Cmd_SearchLand     16  //开环4
#define SDK_Cmd_SearchLand_down 17  //开环4
#define SDK_Cmd_Searchgan      18  //开环4
#define SDK_Cmd_Pos5           19  //开环4

/************
电赛思路分析：

基本要求1：循线1圈、绕B、轨迹在虚线框内（0.6m）     30
基本要求2：  150s内                                 10
基本要求3：发现黄色、距离<=0.3m时声光提示           10
发挥部分1：拍摄条形码图片 存储、显示、识别          5
发挥部分2：拍摄二维码图片 存储、显示、识别          5
发挥部分3：照片<=3张                                5
发挥部分4：配重、自主起飞、1米、悬停10s、+-25cm     10
发挥部分5：随机任务、30分钟、现场编程               20
发挥部分6：其他                                     5

基本  50        发挥 50              报告  20
`
超时30分钟啥也没有   防护眼镜   防护手套


基本元素：
自主起飞、自主降落、寻电线、绕飞、进线、找A、声光提示、安全靠近、退回、
现场编程需要： 偏航角、偏航角速度控制、水平速度、水平位置、

飞行一圈的流程：
起飞、自转找杆、到平行、切入循线、循线、靠近条形码、声光、等待拍照、继续循线、
切入B杆、等待拍二维码、拍不到就转、环绕B杆、平行切入循线、循线、到A、降落
*****************/

void SDK_StateMachine_Init(void)
{
	CurrentSDKState = 0;
	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_DelayWake;
	SDK_StateMachine[CurrentSDKState++] = 3000;
	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_TakeOff;
	SDK_StateMachine[CurrentSDKState++] = 3000;
	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_PosHold;
	SDK_StateMachine[CurrentSDKState++] = 3000;
	

	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Pos1;  //向左转90度
	SDK_StateMachine[CurrentSDKState++] = 3500;
	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Search1;  //搜索目标并且标记杆
	SDK_StateMachine[CurrentSDKState++] =  70000;
//	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Pos2;   //对准目标杆
	SDK_StateMachine[CurrentSDKState++] = 6000;


	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Searchgan;
	SDK_StateMachine[CurrentSDKState++] = 40000;


	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Search0;  //绕杆
	SDK_StateMachine[CurrentSDKState++] =  70000;
	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Pos3;
	SDK_StateMachine[CurrentSDKState++] = 2000;
	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Pos5;
	SDK_StateMachine[CurrentSDKState++] = 2000;
/*************************************************************************************/	//第一排完成	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Search1;  //搜索目标并且标记杆
	SDK_StateMachine[CurrentSDKState++] =  70000;
	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Pos2;   //对准目标杆
	SDK_StateMachine[CurrentSDKState++] = 6000;
	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Searchgan;
	SDK_StateMachine[CurrentSDKState++] = 40000;
	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Search0;  //绕杆
	SDK_StateMachine[CurrentSDKState++] =  70000;
	
/*************************************************************************************/	//第二排完成		
		SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Pos4;
	SDK_StateMachine[CurrentSDKState++] = 2000;
	

	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_SearchLand; //寻找降落点并降落
	SDK_StateMachine[CurrentSDKState++] = 40000;

	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_SearchLand_down; //寻找降落点并降落
	SDK_StateMachine[CurrentSDKState++] = 40000;
//	
	SDK_StateMachine[CurrentSDKState++] = SDK_Cmd_Land;
	SDK_StateMachine[CurrentSDKState++] = 10000;
	
	SDKStateMAX = CurrentSDKState-2;
	CurrentSDKState=0;
}

#define  SDK_Height     1.0f
#define  SDK_Height_H   (SDK_Height+0.05f)
#define  SDK_Height_L   (SDK_Height-0.05f)

float Search0OriginYaw;
void SDK_StateMachine_Loop(void)
{
	static unsigned int LastSDKState;
	switch ( SDK_StateMachine[ CurrentSDKState ] )
	{
		
/*************************************************************************************/	
		case SDK_Cmd_DelayWake:
			if(LastSDKState!=SDK_Cmd_DelayWake)
			{
				Ctrler.locxPID.Des = Ctrler.locxPID.FB;
				Ctrler.locyPID.Des = Ctrler.locyPID.FB;
				Ctrler.yawPID.Des  = Ctrler.yawPID.FB;
				temp_V_x=-1;
				temp_V_y=-1;
				temp_V_h=0;
				temp_Gyroz=0;
				data.TofData.last_high = 0;
				yaw_init =  Ctrler.yawPID.FB; //记录当前偏差
			}
			if(SDK_StateMachine[ CurrentSDKState +1 ] >=20)SDK_DelayWakeFlag=1;
			else SDK_DelayWakeFlag=0;
		
			break;
			
/*************************************************************************************/
		
		case SDK_Cmd_TakeOff:

		  OpenMVData.stm32_flag.buzzer_flag =1;
		  OpenMVData.stm32_flag.buzzer_flag =0;
			Ctrler.Z_posPID.Des = SDK_Height;
			if(Ctrler.Z_posPID.FB >=SDK_Height_L && Ctrler.Z_posPID.FB<=SDK_Height_H)
				SDK_StateMachine[ CurrentSDKState +1 ]=0;
		
			break;
			

/*************************************************************************************/
		case SDK_Cmd_PosHold:
        
		  yaw_SDK =  Ctrler.yawPID.FB-yaw_init;  //记录当前yaw轴的角度，大约为0度
		 
		  OpenMVData.stm32_flag.openmv_flag=0;  //顶部openmv开始工作	
		  OpenMVData.stm32_flag.buzzer_flag =1;
		  OpenMVData.stm32_flag.buzzer_flag =0;
		  OpenMVData.stm32_flag.led_flag =4;
		
				temp_V_x=-1;
				temp_V_y=-1;	
				temp_Gyroz=0;
		
					
			break;
		
/*************************************************************************************/		
		
		case SDK_Cmd_Pos1:
			
		 yaw_SDK =  Ctrler.yawPID.FB-yaw_init;  //记录当前yaw轴的角度
	  if(LastSDKState!=SDK_Cmd_Pos1)
			{
        Ctrler.yawPID.Des -= 50.0f;   //先转90度
				temp_V_x=-1;
				temp_V_y=-1;
			}		
			OpenMVData.stm32_flag.buzzer_flag =0;
      OpenMVData.stm32_flag.openmv_flag=4;  //顶部openmv开始工作	
			yaw_cnt= 0;
			
			break;	
			
/*************************************************************************************/				
			
		case SDK_Cmd_Search1:     //自转寻找杆并且标记
			  
			if(OpenMVData.openmv_top.yaw ==0)
			{
		  time_SDK_cnt++;
			OpenMVData.stm32_flag.buzzer_flag =1;
			}
			else
			{
		 	time_SDK_cnt = 0;
			OpenMVData.stm32_flag.buzzer_flag =0;
			}
			
			yaw_SDK =  Ctrler.yawPID.FB-yaw_init;  //记录当前yaw轴的角度
		   if(yaw_SDK>180)
			 {
			   yaw_SDK = yaw_SDK-360.f;
			 }
			 else if(yaw_SDK<-180.0f)
			 {
			   yaw_SDK = yaw_SDK+360.f;
			 }
		
				temp_V_x=-1;
						
	   		if(OpenMVData.openmv_top.yaw>30)
					temp_Gyroz = 30;
				else if(OpenMVData.openmv_top.yaw <-30)
					temp_Gyroz = -30;
		    else 
			    temp_Gyroz = OpenMVData.openmv_top.yaw*0.9f;
				
				if(OpenMVData.openmv_top.valid ==1&&time_SDK_cnt>80 ) //识别到了
				{
					
				  temp_V_y =-1;
			    temp_V_x=-1;	
					yaw_SDK_openmv[yaw_cnt] = yaw_SDK; // 记录此时杆的数量
					if(yaw_cnt<50)
					{
					yaw_cnt++;
					}
					time_SDK_cnt = 0;
					OpenMVData.stm32_flag.openmv_flag=4; //此处发标志位继续
				}
		

			if(yaw_SDK>50.0f)
			{
			    SDK_StateMachine[ CurrentSDKState +1 ]=0;  //切换任务
				  OpenMVData.stm32_flag.buzzer_flag =1;
					OpenMVData.stm32_flag.openmv_flag=0;
				  last_search_yaw = Ctrler.yawPID.FB;
				 	last_search_yaw = SDK_yaw_gan_cnt(yaw_cnt); //记录杆的方位
			 
			}
	 
			break;
		

			
/*************************************************************************************/				
						
		case SDK_Cmd_Pos2:
			
		 if(LastSDKState!=SDK_Cmd_Pos2)
			{
			temp_V_x=-1;
			temp_V_y=-1;
			temp_Gyroz = 0;
				
			}
		yaw_Des = SDK_yaw_gan_cnt(yaw_cnt)+yaw_init;
		  if(yaw_Des>180.0f)
			yaw_Des = yaw_Des-360.f;
     else if(yaw_Des<-180.0f)			
			 yaw_Des =yaw_Des+360.f;
		 else
			 yaw_Des =yaw_Des;
		 
	    Ctrler.yawPID.Des = yaw_Des; 
	 
     	OpenMVData.stm32_flag.buzzer_flag =0;
			OpenMVData.stm32_flag.openmv_flag=0;
		last_search_yaw = SDK_yaw_gan_cnt(yaw_cnt); //记录杆的方位
	
		
		break;
		
/*************************************************************************************/		
			case SDK_Cmd_Searchgan:     //自转寻找杆
			
			OpenMVData.stm32_flag.openmv_flag=6;
			OpenMVData.stm32_flag.buzzer_flag =0;
				temp_V_x=-1;
						
	   		if(OpenMVData.openmv_top.yaw>30)
					temp_Gyroz = 30;
				else if(OpenMVData.openmv_top.yaw <-30)
					temp_Gyroz = -30;
		    else 
			    temp_Gyroz = OpenMVData.openmv_top.yaw;
				
				if(OpenMVData.openmv_top.yaw ==0&&OpenMVData.openmv_top.valid ==1)  //识别到了
				{
				  temp_V_y =-12;
			    temp_V_x=-1;	
				}
		
		  if(OpenMVData.stm32_flag.stm_to_distance>50&&OpenMVData.stm32_flag.stm_to_distance<80)
	  	{	
				searchcnt++;
	  	}
			if(OpenMVData.openmv_top.x>70&&OpenMVData.openmv_top.x<90)
	  	{	
				searchcnt1++;
	  	}
			if(searchcnt>200&&searchcnt1>100)
			{
			    SDK_StateMachine[ CurrentSDKState +1 ]=0;  //切换任务
				  OpenMVData.stm32_flag.buzzer_flag =1;
					OpenMVData.stm32_flag.openmv_flag=0;
//				  last_search_yaw = Ctrler.yawPID.FB;
				 
			}
	 
			break;
	
/*************************************************************************************/	
		
		case SDK_Cmd_Search0:
      
		  	if(last_search_yaw>0)  //在右侧 顺
			 {	 
			  OpenMVData.stm32_flag.openmv_flag=8;
			 }
			 else if(last_search_yaw<=0)                  //在左侧  逆
			 {
				 OpenMVData.stm32_flag.openmv_flag=7;
			 }
		  
		
		 	yaw_SDK =  Ctrler.yawPID.FB-yaw_init;  //记录当前yaw轴的角度
		   if(yaw_SDK>180)
			 {
			   yaw_SDK = yaw_SDK-360.f;
			 }
			 else if(yaw_SDK<-180.0f)
			 {
			   yaw_SDK = yaw_SDK+360.f;
			 }
			 
			 yaw_Des1 = 65.0f+yaw_init;
			 if(yaw_Des1>180.0f)
				 yaw_Des1 =yaw_Des1-360.0f;
			 else if(yaw_Des1<-180.0f)
				 yaw_Des1 =yaw_Des1+360.0f;
			 else 
				 yaw_Des1 = yaw_Des1;
			 
			 	 
			 yaw_Des2 = -65.0f+yaw_init;
			 if(yaw_Des2>180.0f)
				 yaw_Des2 =yaw_Des2-360.0f;
			 else if(yaw_Des2<-180.0f)
				 yaw_Des2 =yaw_Des2+360.0f;
			 else 
				 yaw_Des2 = yaw_Des2;
			 
		 yawcntsearch++;
		 OpenMVData.stm32_flag.led_flag =1;//红色
		 OpenMVData.stm32_flag.buzzer_flag =0;
//		 OpenMVData.stm32_flag.openmv_flag=7;

		  //绕杆的大概思路  x方向向右匀速运动，Y方向与杆保持恒定的距离，yaw轴通过openmv保持始终对着杆。
	
//		if(OpenMVData.openmv_top.valid ==1)
//		{
			 if(last_search_yaw>0)  //在右侧 顺
			 {
		    temp_V_x = +7.0f;
			 }
			 else if(last_search_yaw<=0)                  //在左侧  逆
			 {
				 temp_V_x = -7.0f;
			 }
		
		   temp_V_y = -0.6*(OpenMVData.stm32_flag.stm_to_distance - 30.0f);
		  
		   if(temp_V_y>8)
				 temp_V_y = 8;
			 else if(temp_V_y<-8)
				  temp_V_y = -8;
			 else
				 temp_V_y = temp_V_y;
//		 }
//		else if(OpenMVData.openmv_top.valid ==0)
//		{
//		  temp_V_x = -1;
//			
//			temp_V_y = -0.6*(OpenMVData.stm32_flag.stm_to_distance - 30.0f);
//			
//			 if(temp_V_y>8)
//				 temp_V_y = 8;
//			 else if(temp_V_y<-8)
//				  temp_V_y = -8;
//			 else
//				 temp_V_y = temp_V_y;
//		}
				if(OpenMVData.openmv_top.yaw>30)
					temp_Gyroz = 30;
				else if(OpenMVData.openmv_top.yaw <-30)
					temp_Gyroz = -30;
		    else 
			    temp_Gyroz = OpenMVData.openmv_top.yaw*1.5;
				
	   	if(last_search_yaw>0)  //在右侧
			{
				
				if(yaw_SDK>70.0f)
				{
					SDK_StateMachine[ CurrentSDKState +1 ]=0;  //切换任务 顺4逆5  顺2逆3
	        OpenMVData.stm32_flag.buzzer_flag =1;
//					Ctrler.locyPID.Des =	Ctrler.locyPID.FB;
//						Ctrler.locxPID.Des =	Ctrler.locxPID.FB;
				 OpenMVData.stm32_flag.openmv_flag=3;
				  temp_V_x=-1;
		    	temp_V_y=-1;
			  
				}
			}
			else if(last_search_yaw<0) //在左侧
				
						if(yaw_SDK<-70.0f)
				{
					SDK_StateMachine[ CurrentSDKState +1 ]=0;  //切换任务
	        OpenMVData.stm32_flag.buzzer_flag =1;

				  OpenMVData.stm32_flag.openmv_flag=2;
				  temp_V_x=-1;
		    	temp_V_y=-1;
			  
				}
				 
			
			break;
				
				
/*************************************************************************************/		
		
		case SDK_Cmd_Pos3:
			
			temp_V_x=-1;
			temp_V_y=-1;
			temp_Gyroz = 0;
	   	yaw_cnt=0;
		  yaw_SDK_openmv[0]=0;
		  yaw_SDK_openmv[1]=0;
		  yaw_SDK_openmv[2]=0;
	  	searchcnt= 0;
	  	searchcnt1=0;
		 
		  	if(last_search_yaw>0)  //在右侧 逆
			 {	 
			  OpenMVData.stm32_flag.openmv_flag=5;
			 }
			 else if(last_search_yaw<=0)                  //在左侧  顺
			 {
				 OpenMVData.stm32_flag.openmv_flag=4;
			 }
		
		break;			
/*************************************************************************************/			
				
			case SDK_Cmd_Pos4:
				
		  temp_V_x=-1;
			temp_V_y=-1;
			temp_Gyroz = 0;
			
     	break;
/*************************************************************************************/				
			
			case SDK_Cmd_Pos5:
			
		 if(LastSDKState!=SDK_Cmd_Pos5)
			{
			temp_V_x=-1;
			temp_V_y=-1;
			temp_Gyroz = 0;		
			}
			
		 	if(last_search_yaw>0)  //在右侧 顺
			 {	 
			   yaw_Des = 45.0f+yaw_init;
		     if(yaw_Des>180.0f)
			   yaw_Des = yaw_Des-360.f;
         else if(yaw_Des<-180.0f)			
			   yaw_Des =yaw_Des+360.f;
		     else
			   yaw_Des =yaw_Des;
			 }
		 
			 else if(last_search_yaw<=0)                  //在左侧  逆
			 {
				 yaw_Des = -45.0f+yaw_init;
		     if(yaw_Des>180.0f)
			   yaw_Des = yaw_Des-360.f;
         else if(yaw_Des<-180.0f)			
			   yaw_Des =yaw_Des+360.f;
		     else
			   yaw_Des =yaw_Des;
			 }

	    Ctrler.yawPID.Des = yaw_Des; 
			 
	 	break;

/*************************************************************************************/		
			
		case SDK_Cmd_SearchLand:     //自转寻找降落点
			
				temp_V_x=-1;
				 OpenMVData.stm32_flag.buzzer_flag =0;
//				if(OpenMVData.openmv_top.y>20)
//					temp_V_y = 20;
//				else if(OpenMVData.openmv_top.y<-20)
//				  temp_V_y = -20;
//				else
//			  temp_V_y= OpenMVData.openmv_top.y;
						
	   		if(OpenMVData.openmv_top.yaw>30)
					temp_Gyroz = 30;
				else if(OpenMVData.openmv_top.yaw <-30)
					temp_Gyroz = -30;
		    else 
			    temp_Gyroz = OpenMVData.openmv_top.yaw*1.8;
				
				if(OpenMVData.openmv_top.yaw ==0&& OpenMVData.BlobData.mag==0)  //识别到了
				{
				  temp_V_y =-15;
			    temp_V_x=-1;	
				}
		
		   	if(OpenMVData.BlobData.mag==1)
	  	{	
				OpenMVData.stm32_flag.openmv_flag=0;
				SDK_StateMachine[ CurrentSDKState +1 ]=0;  //切换任务
				OpenMVData.stm32_flag.buzzer_flag =1;
	  	}
	 
			break;	
			
/*************************************************************************************/					


	case SDK_Cmd_SearchLand_down:     //寻找降落点
										
		if(OpenMVData.BlobData.mag==1)
		{			
			OpenMVData.stm32_flag.buzzer_flag =1;
			temp_Gyroz = 0;	
			
			if(abs(OpenMVData.BlobData.x)>30)
			{
			 temp_V_x = OpenMVData.BlobData.x*0.4f;
			}
			else 
				temp_V_x = OpenMVData.BlobData.x*0.5f;
		

		  	if(temp_V_x>11)
					temp_V_x = 11;
				else if(temp_V_x<-11)
				  temp_V_x = -11;
				else
			  temp_V_x= temp_V_x;
/******************************************/							
				if(abs(OpenMVData.BlobData.y)>30)
			{
			 temp_V_y = OpenMVData.BlobData.y*0.4f;
			}
			else 
				temp_V_y = OpenMVData.BlobData.y*0.5f;	
				
				if(temp_V_y>11)
					temp_V_y = 11;
				else if(temp_V_y<-11)
				  temp_V_y = -11;
				else
			  temp_V_y= temp_V_y;		
		}		
		else if(OpenMVData.BlobData.mag==0)		
		{
		  temp_V_x =0;
			temp_V_y =0;
			temp_Gyroz = 0;		
		}
		
		if(abs(OpenMVData.BlobData.x)<10 && abs(OpenMVData.BlobData.y)<10 &&OpenMVData.BlobData.mag==1)
		{
		   landcnt1++;
		}
		if(landcnt1>150)
			SDK_StateMachine[ CurrentSDKState +1 ]=0;
			 
			break;	
			
/*************************************************************************************/				
			
			case SDK_Cmd_Land:
						
			
			if(OpenMVData.BlobData.mag==1&&Ctrler.Z_posPID.FB>0.4f)
		{			
			temp_Gyroz = 0;	
			
			if(abs(OpenMVData.BlobData.x)>20)
			{
			 temp_V_x = OpenMVData.BlobData.x*0.4f;
			}
			else 
				temp_V_x = OpenMVData.BlobData.x*0.4f;
		

		  	if(temp_V_x>8)
					temp_V_x = 8;
				else if(temp_V_x<-8)
				  temp_V_x = -8;
				else
			  temp_V_x= temp_V_x;
/*************************************************************/							
				if(abs(OpenMVData.BlobData.y)>20)
			{
			 temp_V_y = OpenMVData.BlobData.y*0.4f;
			}
			else 
				temp_V_y = OpenMVData.BlobData.y*0.4f;	
				
				if(temp_V_y>8)
					temp_V_y = 8;
				else if(temp_V_y<-8)
				  temp_V_y = -8;
				else
			  temp_V_y= temp_V_y;		
		}		
//			else 	
//		{
//		  temp_V_x =0;
//			temp_V_y =0;	
//		}
//		  temp_V_x =-1;
//			temp_V_y =-1;
			OpenMVData.stm32_flag.openmv_flag=0;  //顶部openmv开始工作	
			OpenMVData.stm32_flag.buzzer_flag =0;
		
//			Ctrler.Z_posPID.Des = 0;
			if(Ctrler.Z_posPID.FB>0.5f)  //不同高度范围内设置不同的降落速度
				temp_V_h = -0.04;
			else if(Ctrler.Z_posPID.FB>=0&&Ctrler.Z_posPID.FB<=0.5f)
				temp_V_h = -0.35;
//			else if(Ctrler.Z_posPID.FB>0&&Ctrler.Z_posPID.FB<0.2f)
//			  temp_V_h = -0.4;
			
			OpenMVData.stm32_flag.led_flag=1; //红色
			if(LastSDKState!=SDK_Cmd_Land)
			{
//				x_test = 0;
//				y_test = 0;
//				yaw_test =0;
				landcnt=0;
//				Ctrler.locyPID.Des = Ctrler.locyPID.FB;
//				Ctrler.locxPID.Des = Ctrler.locxPID.FB;
			}
			if(Ctrler.Z_posPID.FB <=0.13f)
			{
				if(landcnt<=50)SDKLandFlag=1;
				else 
				{
					SDKLandFlag=0;
					SDK_StateMachine[ CurrentSDKState +1 ]=0;
					KeySDKflag=0;
					DroneStatus.ARM_Status = DisArmed;
				}
				landcnt++;

			}
			break;
			
		default:
			break;
	}
	
	LastSDKState = SDK_StateMachine[ CurrentSDKState ];
	
//	if(SDK_StateMachine[ CurrentSDKState +1 ]<=1000 && SDK_StateMachine[CurrentSDKState +1 ]>=10)\
//						BEEP_ON
//	else BEEP_OFF;
	if(SDK_StateMachine[ CurrentSDKState +1 ]<=1000 && SDK_StateMachine[CurrentSDKState +1 ]>=10)\
						SDK_StateChangeFlag=1;
	else SDK_StateChangeFlag=0;
	
	
	if(SDK_StateMachine[ CurrentSDKState +1 ]>=5) SDK_StateMachine[ CurrentSDKState +1 ] -=5;
	if(SDK_StateMachine[ CurrentSDKState +1 ]<=10 && CurrentSDKState<SDKStateMAX)CurrentSDKState+=2;
}

void SDK_Set_V_Loc(void)
{
	
///**********************************控制方案1***********************************************/	
//	if(
//			 SDK_StateMachine[ CurrentSDKState ]==  SDK_Cmd_Pos1
//			   || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos2
//			      || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos3        
//		) 
//	{
//		if(SBUS_CH_VALID(PITCH_CH))//打杆对应期望水平速度
//				Ctrler.locysPID.Des = -((Remoter.PitCtrler-3000)/1000.0)*Stick_to_MAX_Horizontal_Rate;
//		else if(temp_V_y!= -1)	Ctrler.locysPID.Des = temp_V_y;			
//				
//		if(SBUS_CH_VALID(ROLL_CH))//打杆对应期望水平速度
//					Ctrler.locxsPID.Des = -((Remoter.RolCtrler-3000)/1000.0)*Stick_to_MAX_Horizontal_Rate;  
//		else if(temp_V_x != -1)  Ctrler.locxsPID.Des = temp_V_x;
//		SDK_Cmd_Searchgan
//	}
/*********************************控制方案2**********************************************/		
	if(
		   SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Search0
	       || SDK_StateMachine[ CurrentSDKState ]==  SDK_Cmd_Pos1
	         || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos2
  		      || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos3  
	           || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos4 
                || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos5  	
	              || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Searchgan 
	            || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_SearchLand 
	              || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_SearchLand_down 
	                 || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Land 
	                      || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Search1 
		)
	{
		if(SBUS_CH_VALID(PITCH_CH))//打杆对应期望水平速度
				Ctrler.locysPID.Des = -((Remoter.PitCtrler-3000)/1000.0)*Stick_to_MAX_Horizontal_Rate;
		else if(temp_V_y== -1) 
		{  
			if(y_test ==0) //只进入一次
			{
			Ctrler.locyPID.Des =Ctrler.locyPID.FB;
				y_test=1;
			}
			else if(y_test ==1)
			Ctrler.locysPID.Des = Ctrler.locyPID.U;
		}
		
		else if (temp_V_y != -1) 
		{
			Ctrler.locysPID.Des = temp_V_y;
				y_test=0;
		}
	
		if(SBUS_CH_VALID(ROLL_CH))//打杆对应期望水平速度
					Ctrler.locxsPID.Des = -((Remoter.RolCtrler-3000)/1000.0)*Stick_to_MAX_Horizontal_Rate;  
		else if(temp_V_x == -1)
		{ 
			if(x_test ==0) //只进入一次
			{
			Ctrler.locxPID.Des =Ctrler.locxPID.FB;
			x_test=1;	
			}
			else if(x_test ==1)
			Ctrler.locxsPID.Des = Ctrler.locxPID.U;
		}	
		else if(temp_V_x!= -1)
		{
			Ctrler.locxsPID.Des = temp_V_x;
			x_test =0;
		}
	}

}



void SDK_Set_Gyroz(void)
{
	if(SDK_StateMachine[ CurrentSDKState ]==SDK_Cmd_FollowLine
			|| SDK_StateMachine[ CurrentSDKState ]==  SDK_Cmd_Surround
		   	|| SDK_StateMachine[ CurrentSDKState ]==  SDK_Cmd_PowerLine
		 	|| SDK_StateMachine[ CurrentSDKState ]==  SDK_Cmd_Search0
				|| SDK_StateMachine[ CurrentSDKState ]==  SDK_Cmd_Pos1
			|| SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos2
				|| SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos3
	        || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos4  
	       || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos5  
	        || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Searchgan 
	      || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_SearchLand 
	        || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_SearchLand_down 
	           || SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Search1 
	
	   )
	{
		if(SBUS_CH_VALID(YAW_CH))
			Ctrler.gyrozPID.Des =  ((Remoter.YawCtrler-3000)/1000.0)*Stick_to_MAX_GyroZ ;
		else if(temp_Gyroz == 0)
		{
			if(yaw_test==0) //标志位，只进入一次
			{
			Ctrler.yawPID.Des   = Ctrler.yawPID.FB;
				yaw_test =1;
			}
			else if(yaw_test==1)
			{			
				if(Ctrler.yawPID.U>25.0f)
					Ctrler.gyrozPID.Des  = 25.0f;
				else if(Ctrler.yawPID.U< -25.0f)
						Ctrler.gyrozPID.Des  = -25.0f;
				else
					Ctrler.gyrozPID.Des = Ctrler.yawPID.U ;				
			}
		}
		else
		{
			Ctrler.gyrozPID.Des = temp_Gyroz ;
		  yaw_test=0;
		}
	}				
}

void SDK_Set_H_Loc(void)
{
	
		if(SDK_StateMachine[ CurrentSDKState ]==SDK_Cmd_Land )
		{
			
	  if(SBUS_CH_VALID(THR_CH))
 				Ctrler.Z_ratePID.Des =  ((Remoter.ThrCtrler-3000)/1000.0)*Stick_to_MAX_V_height ;
		else if(temp_V_h!=0)
				Ctrler.Z_ratePID.Des =temp_V_h;
	  }
}

void SDK_Set_Yaw(void)
{
//if
//	(SDK_StateMachine[ CurrentSDKState ]==SDK_Cmd_FollowLine
//			|| SDK_StateMachine[ CurrentSDKState ]==  SDK_Cmd_Surround
//		   	|| SDK_StateMachine[ CurrentSDKState ]==  SDK_Cmd_PowerLine
//		 	|| SDK_StateMachine[ CurrentSDKState ]==  SDK_Cmd_Search0
//				|| SDK_StateMachine[ CurrentSDKState ]==  SDK_Cmd_Pos1
//			|| SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos2
//				|| SDK_StateMachine[ CurrentSDKState ]== SDK_Cmd_Pos3
//	
//	 )
//	{
//		 if(temp_Gyroz!=0)
//			Ctrler.yawPID.Des = Ctrler.yawPID.Des+ temp_Gyroz;
//		 else
//			 Ctrler.yawPID.Des =Ctrler.yawPID.FB;
//	}		
}

float   yaw_SDK_openmv_E[100];
float SDK_yaw_gan_cnt(u8 yawcnt0)
{
   if(yawcnt0 == 1)
	 {
	   return yaw_SDK_openmv[0];
	 }
	 else if(yawcnt0 == 2)
	 { 
	   return yaw_SDK_openmv[0];
	 }
	 else if(yawcnt0 == 3)
	 {
	  yaw_SDK_openmv_E[0]=yaw_SDK_openmv[1]-yaw_SDK_openmv[0];
	  yaw_SDK_openmv_E[1]=yaw_SDK_openmv[2]-yaw_SDK_openmv[1];
		 if(yaw_SDK_openmv_E[0]>yaw_SDK_openmv_E[1])
		 {
			 return  yaw_SDK_openmv[0];
		 }
		 else if(yaw_SDK_openmv_E[0]<yaw_SDK_openmv_E[1])
		 {
		  return  yaw_SDK_openmv[2];
		 }
	 }
	 else 
		 return 0;
	
}


void SDK_Set_Pos_Loc(void)
{
	
}

void SDK_StateMachine_Reset(void)
{
	//if( CurrentSDKState !=0 ) KeySDKflag=0;
	CurrentSDKState = 0;
	SDK_StateMachine_Init();
	ShotQRCodeFlag=0;
}
