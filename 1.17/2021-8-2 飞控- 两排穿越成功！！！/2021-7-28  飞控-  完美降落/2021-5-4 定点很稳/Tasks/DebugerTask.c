#include "DebugerTask.h"
#include "delay.h"
#include "config&param.h"
#include "uart2_dma.h"
#include "uart5_dma.h"
#include "uart6_dma.h"
#include "uart7_dma.h"
#include "anoV65.h"
#include "x_wdp.h"
#include "StabilizerTask.h"
#include "pid.h"
#include "RemoterTask.h"
#include "us_100.h"
#include "optical.h"
#include "tf_mini_plus.h"
#include "openmv.h"


void USART_data_deal(UCHAR8 CmdFlag);
void imu_Get_One_Byte(unsigned char datax);

void DebugerTask(void *pvParameters)
{

	u32 lastWakeTime = getSysTickCnt();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 5);

		ANO_V65_Report(ANO_FUNC_UserData1);//  串口7  无线串口
		 
	  ANO_Report_Status(); //串口4 云控
		
//		ANO_Report_Sensor();  //串口5 视觉
		}

}


UartManageTypeDef   UartManager;
void UartRxDecodeTask(void *pvParameters)
{
	int i;
	u32 tick=0;
	u32 lastWakeTime = getSysTickCnt(); 
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 1 );		/* 1tick */ 
		
	//pitch -4   roll +178.8
//		      G_ST_Vision.Send.Acc_X_Real  = Acc_X_Real;
//	      G_ST_Vision.Send.Acc_Y_Real  = Acc_Y_Real;
//	      G_ST_Vision.Send.Acc_Z_Real  = Acc_Z_Real;
	
		for(i=0;i<50;i++)
		imu_Get_One_Byte(UART4_RX_BUFA[i]);
		
		unAimData.stEnemyE.Pitchangle = unAimData.stEnemyE.Pitchangle -1.1f;
		unAimData.stEnemyE.rollangle  = unAimData.stEnemyE.rollangle +179.6f;
//		unAimData.stEnemyE.Yawangle   = unAimData.stEnemyE.Yawangle -187.3f;
		
		if(unAimData.stEnemyE.rollangle>180)
		{
		  unAimData.stEnemyE.rollangle=unAimData.stEnemyE.rollangle-360.0f;
		}
		if(unAimData.stEnemyE.Yawangle>0)
		{
		  unAimData.stEnemyE.Yawangle=unAimData.stEnemyE.Yawangle-360.0f;
		}
		if(unAimData.stEnemyE.Yawangle<-180.0f)
		{
		  unAimData.stEnemyE.Yawangle=unAimData.stEnemyE.Yawangle+360.0f;
		}

		unAimData.stEnemyE.Acc_X_Real=unAimData.stEnemyE.Acc_X_Real; //后正
		
		unAimData.stEnemyE.Acc_Y_Real=unAimData.stEnemyE.Acc_Y_Real;//左正
		
		
		
		if(unAimData.stEnemyE.Acc_X_Real>50&&unAimData.stEnemyE.Acc_X_Real<1200)
		{
		OpenMVData.stm32_flag.stm_to_distance = unAimData.stEnemyE.Acc_X_Real*0.1f;  //超声波数据
		}
		
			
		//处理数据
		OpenMVData.openmv_top.x_raw = unAimData.stEnemyE.flag[0];
		OpenMVData.openmv_top.y_raw = unAimData.stEnemyE.flag[1];
		OpenMVData.openmv_top.yaw_raw = unAimData.stEnemyE.flag[2];
		OpenMVData.openmv_top.key = unAimData.stEnemyE.flag[3];
		OpenMVData.openmv_top.valid = unAimData.stEnemyE.openmv_valid;
		
//		if(OpenMVData.openmv_top.x_raw >120)
//			OpenMVData.openmv_top.x_raw = OpenMVData.openmv_top.x_raw -256;
		if(OpenMVData.openmv_top.y_raw  >120)
			OpenMVData.openmv_top.y_raw  = OpenMVData.openmv_top.y_raw  -256;
		if(OpenMVData.openmv_top.yaw_raw  >120)
      OpenMVData.openmv_top.yaw_raw  = OpenMVData.openmv_top.yaw_raw  -256;
		
		OpenMVData.openmv_top.x =OpenMVData.openmv_top.x_raw;
		OpenMVData.openmv_top.y=OpenMVData.openmv_top.y_raw;
		OpenMVData.openmv_top.yaw=OpenMVData.openmv_top.yaw_raw;
		
		Uart1_Tick_RX();
		if(Uart1_RX_POS)
		{
			for(i=0;i<Uart1_RX_POS;i++)AnoOF_GetOneByte(USART1_RX_BUF[i]);
			Uart1_RX_POS=0;
		}
		
		Uart6_Tick_RX();
		if(Uart6_RX_POS)
		{
			for(i=0;i<Uart6_RX_POS;i++)TF_Mini_Plus_Get_One_Byte(UART6_RX_BUF[i]);
			Uart6_RX_POS=0;
		}
		
		Uart5_Tick_RX();
		if(Uart5_RX_POS)
		{
			for(i=0;i<Uart5_RX_POS;i++)OpenMV_Get_1_Byte(UART5_RX_BUF[i]);
			Uart5_RX_POS=0;
		}
		
		if(tick%9==0)
		{
			//不失控：失控 = 13:7   失控8秒恢复
			UartManager.Uart2_RX_CNT_ThisTick = uDMAChannelSizeGet( UDMA_PRI_SELECT |UDMA_CHANNEL_UART2RX);
			if(
				UartManager.Uart2_RX_CNT_ThisTick == UartManager.Uart2_RX_CNT_LastTick&&  \
				 UartManager.Uart2_RX_CNT_ThisTick >0 &&
			UartManager.Uart2_RX_CNT_ThisTick<UART2_RXBUF_SIZE&& 
			UART2_RX_BUFA[25]!=0x0F  
			)
			{
					ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART2RX | UDMA_PRI_SELECT,
																		 UDMA_MODE_BASIC,
																		 (void *)(UART2_BASE + UART_O_DR),
																		 UART2_RX_BUFA, sizeof(UART2_RX_BUFA));							 
				ROM_uDMAChannelEnable(UDMA_CHANNEL_UART2RX);
			}
			UartManager.Uart2_RX_CNT_LastTick = UartManager.Uart2_RX_CNT_ThisTick;
		}
		
		if(tick%5==0)
		{
			UartManager.Uart7_RX_CNT_ThisTick = uDMAChannelSizeGet( UDMA_CHANNEL_UART7RX | UDMA_PRI_SELECT);
			if(UartManager.Uart7_RX_CNT_ThisTick == UartManager.Uart7_RX_CNT_LastTick  \
				//&& UartManager.Uart7_RX_CNT_ThisTick >0 
			&& UartManager.Uart7_RX_CNT_ThisTick<UART7_RXBUF_SIZE
			//	&& UART2_RX_BUFA[25]!=0x0F  
			)
			{
				Uart7_Rec_CNT = UART7_RXBUF_SIZE -  UartManager.Uart7_RX_CNT_ThisTick;
				for(i=0;i<Uart7_Rec_CNT;i++)UART7_RX_BUF[i]=UART7_RX_BUFA[i];
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART7RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(UART7_BASE + UART_O_DR),
                                   UART7_RX_BUFA, sizeof(UART7_RX_BUFA));
																	 
				ROM_uDMAChannelEnable(UDMA_CHANNEL_UART7RX);
				
				for(i=0;i<Uart7_Rec_CNT;i++)ANO_CMD_GetOneByte(UART7_RX_BUF[i]);
			}
			UartManager.Uart7_RX_CNT_LastTick = UartManager.Uart7_RX_CNT_ThisTick;
		}
		
		
		tick++;
	}
}


void Send_Data_WDP(void)
{
//	static unsigned char i;
//	short datatx[10];
//	
//	WDP_T.pbuf=UART2_TX_BUF;
//	WDP_T.number_of_data=7;
//	WDP_T.pdata=datatx;
//	
////	datatx[0]=data.OpticalData.loc_x*1.0f;
////	datatx[1]=data.OpticalData.loc_y*1.0f;
////	datatx[2]=data.OpticalData.loc_xs*100.0f;
////	datatx[3]=data.OpticalData.loc_ys*100.0f;
////	datatx[4]=Ctrler.Z_posPID.FB*100.0f;
////	datatx[5]=Ctrler.pitchPID.Des*100.0f;
////	datatx[6]=Ctrler.rollPID.Des*100.0f;
//	
//	x_wdp_encode(&WDP_T);
//	if(i==2)
//	{
//		USART2_DMA_TX_Enable(WDP_T.buf_length);
//		i=0;
//	}
//	
//	i++;
}


/////////////////以下为陀螺仪代码//////////////////////////////////

#define COMM6_RX_FREE_1         0
#define COMM6_RX_FREE_2         1
#define COMM6_RX_FREE_3			      2
#define COMM6_RX_FREE_4         3
#define COMM6_RX_FLAG           4
#define COMM6_RX_END            5


UCHAR8 Visual_Data_RxBuffer[44] = {0};
unsigned char imuStateMachine=0;
unsigned int imuFrameCnt=0,imuValidCnt=0;
void imu_Get_One_Byte(unsigned char datax)
{
  	static UCHAR8 Comm6_Rx_Status = COMM6_RX_FREE_1;
	static UCHAR8 ucPit = 0, ucData = 0, CmdFlag = 0;


		ucData = datax;
		switch(Comm6_Rx_Status)
		{
			case COMM6_RX_FREE_1:
				if(ucData == 0x55)
				{																  
					Comm6_Rx_Status = COMM6_RX_FREE_2;//自由状态下接到0x55认为开始
				}
			break;
				
			case COMM6_RX_FREE_2:
				if(ucData == 0x00)
				{																  
					Comm6_Rx_Status = COMM6_RX_FREE_3;	//表示接收到帧头开始接受数据
						ucPit = 0;
				}
				else
				{
					Comm6_Rx_Status = COMM6_RX_FREE_1;//没有接受到帧头，重新等待接受数据
				}
				
			break;
				
			case COMM6_RX_FREE_3:
				if(ucData == 0x02)
				{																  
					Comm6_Rx_Status = COMM6_RX_FREE_4;	//表示接收到帧头开始接受数据
						ucPit = 0;
				}
				else
				{
					Comm6_Rx_Status = COMM6_RX_FREE_1;//没有接受到帧头，重新等待接受数据
				}
				
			break;	
      case COMM6_RX_FREE_4:
				if(ucData == 0x08)
				{																  
					Comm6_Rx_Status = COMM6_RX_FLAG;	//表示接收到帧头开始接受数据
						ucPit = 0;
				}
				else
				{
					Comm6_Rx_Status = COMM6_RX_FREE_1;//没有接受到帧头，重新等待接受数据
				}
				
			break;			
				
			case COMM6_RX_FLAG:					             
			
					 if(ucPit < 44)
						{
							*(Visual_Data_RxBuffer + ucPit) = ucData;
							ucPit++;
						}
						else
						{
							if(ucData == 0x00)
							{
								Comm6_Rx_Status = COMM6_RX_END;
							}
							else
							{
								Comm6_Rx_Status = COMM6_RX_FREE_1;
							}
						}	
		  break;
						

				
			case COMM6_RX_END:
			 {
					if(ucData == 0xAA)//如果接到了0xAA，数据有效		
					{
						USART_data_deal(CmdFlag);
					}
					Comm6_Rx_Status = COMM6_RX_FREE_1;
				}
			break;
				
			default:
				
			break;
		}  
}
	


void USART_data_deal(UCHAR8 CmdFlag)
{
	for(int i = 0; i < 11 * 4; i++)
	{
		unAimData.ucEData[i] = Visual_Data_RxBuffer[i];
	}
}



