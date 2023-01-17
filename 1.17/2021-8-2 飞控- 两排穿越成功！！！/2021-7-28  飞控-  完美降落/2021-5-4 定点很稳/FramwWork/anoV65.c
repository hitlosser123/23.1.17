#include "anoV65.h"
#include "uart7_dma.h"
#include "StabilizerTask.h"//   data
#include "pid.h"//   ctrler
#include "drv_eeprom.h"
#include "RemoterTask.h"
#include "us_100.h"
#include "openmv.h"
#include "sbus.h"
#include "height.h"
#include "uart5_dma.h"
#include "AutoflyTask.h"

#define  ANO_TX_Enable(len)    USART7_DMA_TX_Enable(len)
#define  ANO_TX_BUF     UART7_TX_BUF
//2019-7-28:  PID修改标志位变为4字节，TM4C123G EEPROM读2字节出错
//   1000x
float ANO_PID_Table[ANO_PID_Table_Num];//  0reserved   1for P 2for I   3for D
unsigned char Need_to_Param_Back;

void ANO_EEPROM_Write(unsigned short* WriteAddr,unsigned char* buf,unsigned short buflen)
{
	EEPROMProgram((u32*)buf, (u32)WriteAddr, buflen);
}

void ANO_EEPROM_Read(unsigned short ReadAddr,unsigned char* buf,unsigned short buflen)
{
	EEPROMRead((u32*)buf, (u32)ReadAddr, buflen);
}

void ANO_Save_All_PID_Table_To_EEPROM(void)
{
	int pidTable[ANO_PID_Table_Num];
	int i;
	for(i=0;i<ANO_PID_Table_Num;i++)pidTable[i]=ANO_PID_Table[i]*1000.0f;
	ANO_EEPROM_Write((u16*)ANO_PID_Addr_in_EEPROM,(unsigned char*)pidTable,ANO_PID_Table_Num*4);
}

void ANO_Read_All_EEPROM_To_PID_Table(void)
{
	unsigned int pidTable[ANO_PID_Table_Num];
	int i;
	ANO_EEPROM_Read(ANO_PID_Addr_in_EEPROM,(unsigned char*)pidTable,ANO_PID_Table_Num*4);
	for(i=0;i<ANO_PID_Table_Num;i++)ANO_PID_Table[i]=pidTable[i]/1000.0f;
}

#define  PID_Status_Changed    0xAA
void Update_Using_PID_from_EEPROM(void)
{
	unsigned short temp;
	ANO_EEPROM_Read(0,(u8*)&temp,2);
	if(temp==PID_Status_Changed)
	{
		ANO_Read_All_EEPROM_To_PID_Table();
		Update_Using_PID_from_Table();
	}
}

unsigned short Get_EEPROM_PID_Status(void)
{
	unsigned int temp=0;
	ANO_EEPROM_Read(0,(u8*)&temp,4);
	return temp;
}

void Save_Using_PID_to_EEPROM(void)
{
	unsigned int temp;
	temp=PID_Status_Changed;
	ANO_EEPROM_Write(0,(u8*)&temp,4);
	Update_Table_From_Using_PID();
	ANO_Save_All_PID_Table_To_EEPROM();
}

void PID_Init(void)
{
	if(Get_EEPROM_PID_Status()==PID_Status_Changed)
	{
		Update_Using_PID_from_EEPROM();
	}
	else
	{
		Save_Using_PID_to_EEPROM();
	}
}

void Update_Using_PID_from_Table(void)
{
	//  123  vRoll  vPitch  vYaw      456 roll pitch   yaw   78 vh h
	//   9 10  vOptical  Optical    11 12  vGPS  GPS
//	Ctrler.gyroxPID.Kp = ANO_PID_Table[1];
//	Ctrler.gyroxPID.Ki = ANO_PID_Table[2];
//	Ctrler.gyroxPID.Kd = ANO_PID_Table[3];
//	Ctrler.gyroyPID.Kp = ANO_PID_Table[4];
//	Ctrler.gyroyPID.Ki = ANO_PID_Table[5];
//	Ctrler.gyroyPID.Kd = ANO_PID_Table[6];
//	Ctrler.gyrozPID.Kp = ANO_PID_Table[7];
//	Ctrler.gyrozPID.Ki = ANO_PID_Table[8];
//	Ctrler.gyrozPID.Kd = ANO_PID_Table[9];
//	
//	Ctrler.rollPID.Kp = ANO_PID_Table[10];
//	Ctrler.rollPID.Ki = ANO_PID_Table[11];
//	Ctrler.rollPID.Kd = ANO_PID_Table[12];
//	Ctrler.pitchPID.Kp = ANO_PID_Table[13];
//	Ctrler.pitchPID.Ki = ANO_PID_Table[14];
//	Ctrler.pitchPID.Kd = ANO_PID_Table[15];
//	Ctrler.yawPID.Kp = ANO_PID_Table[16];
//	Ctrler.yawPID.Ki = ANO_PID_Table[17];
//	Ctrler.yawPID.Kd = ANO_PID_Table[18];
//	
//	Ctrler.Z_ratePID.Kp = ANO_PID_Table[19] *100.0f;//65535
//	Ctrler.Z_ratePID.Ki = ANO_PID_Table[20];
//	Ctrler.Z_ratePID.Kd = ANO_PID_Table[21];
//	Ctrler.Z_posPID.Kp = ANO_PID_Table[22];
//	Ctrler.Z_posPID.Ki = ANO_PID_Table[23];
//	Ctrler.Z_posPID.Kd = ANO_PID_Table[24];	
//	
//	Ctrler.locxsPID.Kp = ANO_PID_Table[25];
//	Ctrler.locxsPID.Ki = ANO_PID_Table[26];
//	Ctrler.locxsPID.Kd = ANO_PID_Table[27];
//	Ctrler.locysPID.Kp = Ctrler.locxsPID.Kp;
//	Ctrler.locysPID.Ki = Ctrler.locxsPID.Ki;
//	Ctrler.locysPID.Kd = Ctrler.locxsPID.Kd;
//	
//	Ctrler.locxPID.Kp = ANO_PID_Table[28];
//	Ctrler.locxPID.Ki = ANO_PID_Table[29];
//	Ctrler.locxPID.Kd = ANO_PID_Table[30];	
//	Ctrler.locyPID.Kp = Ctrler.locxPID.Kp;
//	Ctrler.locyPID.Ki = Ctrler.locxPID.Ki;
//	Ctrler.locyPID.Kd = Ctrler.locxPID.Kd;
}

void Update_Table_From_Using_PID(void)
{
	//  123  vRoll  vPitch  vYaw      456 roll pitch   yaw   78 vh h
	//   9 10  vOptical  Optical    11 12  vGPS  GPS
	ANO_PID_Table[1] = Ctrler.gyroxPID.Kp;
	ANO_PID_Table[2] = Ctrler.gyroxPID.Ki;
	ANO_PID_Table[3] = Ctrler.gyroxPID.Kd;
	ANO_PID_Table[4] = Ctrler.gyroyPID.Kp;
	ANO_PID_Table[5] = Ctrler.gyroyPID.Ki;
	ANO_PID_Table[6] = Ctrler.gyroyPID.Kd;
	ANO_PID_Table[7] = Ctrler.gyrozPID.Kp;
	ANO_PID_Table[8] = Ctrler.gyrozPID.Ki;
	ANO_PID_Table[9] = Ctrler.gyrozPID.Kd;
	
	ANO_PID_Table[10] = Ctrler.rollPID.Kp;
	ANO_PID_Table[11] = Ctrler.rollPID.Ki;
	ANO_PID_Table[12] = Ctrler.rollPID.Kd;
	ANO_PID_Table[13] = Ctrler.pitchPID.Kp;
	ANO_PID_Table[14] = Ctrler.pitchPID.Ki;
	ANO_PID_Table[15] = Ctrler.pitchPID.Kd;
	ANO_PID_Table[16] = Ctrler.yawPID.Kp;
	ANO_PID_Table[17] = Ctrler.yawPID.Ki;
	ANO_PID_Table[18] = Ctrler.yawPID.Kd;
	
	ANO_PID_Table[19] = Ctrler.Z_ratePID.Kp  /100.0f; //65535
	ANO_PID_Table[20] = Ctrler.Z_ratePID.Ki;
	ANO_PID_Table[21] = Ctrler.Z_ratePID.Kd;
	ANO_PID_Table[22] = Ctrler.Z_posPID.Kp;
	ANO_PID_Table[23] = Ctrler.Z_posPID.Ki;
	ANO_PID_Table[24] = Ctrler.Z_posPID.Kd;	
	
	ANO_PID_Table[25] = Ctrler.locxsPID.Kp;
	ANO_PID_Table[26] = Ctrler.locxsPID.Ki;
	ANO_PID_Table[27] = Ctrler.locxsPID.Kd;
	
	ANO_PID_Table[28] = Ctrler.locxPID.Kp;
	ANO_PID_Table[29] = Ctrler.locxPID.Ki;
	ANO_PID_Table[30] = Ctrler.locxPID.Kd;	
}

void  ANO_V65_Report(unsigned char SelectFunc)
{
	switch (SelectFunc)
	{
		case ANO_FUNC_Status:
			ANO_Report_Status();
			break ;
		
		case ANO_FUNC_Sensor:
			ANO_Report_Sensor();
			break;
		
		case ANO_FUNC_RCdata:
			ANO_Report_RCdata();
			break;
		
		case ANO_FUNC_Motor:
			ANO_Report_Motor();
			break;
		
		case ANO_FUNC_Sensor2:
			ANO_Report_Sensor2();
			break;

		case ANO_FUNC_UserData1:
			ANO_Report_UserData1();
			break;
		
		case Report_Param_Back:
			ANO_Report_Param_Back();
			break;
		
		default :
			break;
	}
}

void ANO_Report_Status(void)
{
	unsigned char databuf[50];
	unsigned char BufLen=0;

	float senddata[10];
	unsigned int i;
	
	senddata[0] = OpenMVData.stm32_flag.led_flag; //灯光标志位
	senddata[1] = OpenMVData.stm32_flag.buzzer_flag;;  //蜂鸣器标志位
	senddata[2] = OpenMVData.stm32_flag.openmv_flag;;          //视觉标志位1


	databuf[BufLen++]= 0x55;
	databuf[BufLen++]= 0x00;
  databuf[BufLen++]= 0x02;
	databuf[BufLen++]= 0x03;
	
	databuf[BufLen++]= BYTE0(senddata[0]) ;//   
	databuf[BufLen++]= BYTE1(senddata[0]) ;//   
	databuf[BufLen++]= BYTE2(senddata[0]) ;//    
	databuf[BufLen++]= BYTE3(senddata[0]) ;// 
	
	databuf[BufLen++]= BYTE0(senddata[1]) ;//    
	databuf[BufLen++]= BYTE1(senddata[1]) ;//    
	databuf[BufLen++]= BYTE2(senddata[1]) ;//   
	databuf[BufLen++]= BYTE3(senddata[1]) ;//  
	
	databuf[BufLen++]= BYTE0(senddata[2]) ;//    
	databuf[BufLen++]= BYTE1(senddata[2]) ;//    
	databuf[BufLen++]= BYTE2(senddata[2]) ;//    
	databuf[BufLen++]= BYTE3(senddata[2]) ;//   
	
	
	databuf[BufLen++]=  0x00 ;//    
	databuf[BufLen++]=  0xAA ;//  

	for(i=0;i<BufLen;i++)UART4_TX_BUF[i] = databuf[i];
	USART4_DMA_TX_Enable(BufLen);
}

void ANO_Report_Sensor(void)
{
	unsigned char databuf[50];
	unsigned char BufLen=0;

	float senddata[10];
	unsigned int i;
	
	senddata[0] = 1; //灯光标志位
	senddata[1] = 2;  //蜂鸣器标志位
	senddata[2] = 3;          //视觉标志位1


	databuf[BufLen++]= 0x55;
	databuf[BufLen++]= 0x00;
  databuf[BufLen++]= 0x02;
	databuf[BufLen++]= 0x03;
	
	databuf[BufLen++]= BYTE0(senddata[0]) ;//   
	databuf[BufLen++]= BYTE1(senddata[0]) ;//   
	databuf[BufLen++]= BYTE2(senddata[0]) ;//    
	databuf[BufLen++]= BYTE3(senddata[0]) ;// 
	
	databuf[BufLen++]= BYTE0(senddata[1]) ;//    
	databuf[BufLen++]= BYTE1(senddata[1]) ;//    
	databuf[BufLen++]= BYTE2(senddata[1]) ;//   
	databuf[BufLen++]= BYTE3(senddata[1]) ;//  
	
	databuf[BufLen++]= BYTE0(senddata[2]) ;//    
	databuf[BufLen++]= BYTE1(senddata[2]) ;//    
	databuf[BufLen++]= BYTE2(senddata[2]) ;//    
	databuf[BufLen++]= BYTE3(senddata[2]) ;//   
	
	
	databuf[BufLen++]=  0x00 ;//    
	databuf[BufLen++]=  0xAA ;//  

	for(i=0;i<BufLen;i++)UART5_TX_BUF[i] = databuf[i];
	USART5_DMA_TX_Enable(BufLen);
}

void ANO_Report_RCdata(void)
{
//	unsigned char databuf[50];
//	unsigned char BufLen=0;
//	
//	//acc gyro mag    xyz
//	unsigned short rcdata[10];
//	unsigned int i;
//	
//	//   ANO  rc   range   1000-2000!!!!!!!!!!!!
//	rcdata[0]=Remoter.ThrCtrler/2;//THR
//	rcdata[1]=Remoter.YawCtrler/2;//YAW
//	rcdata[2]=Remoter.RolCtrler/2;//ROLL
//	rcdata[3]=Remoter.PitCtrler/2;//PITCH
//	
//	rcdata[4]=2000;//AUX1
//	rcdata[5]=2500;//AUX2
//	rcdata[6]=3000;//AUX3
//	rcdata[7]=3500;//AUX4
//	rcdata[8]=3700;//AUX5
//	rcdata[9]=4000;//AUX6
//	
//	databuf[BufLen++]=ANO_Frame_Header;
//	databuf[BufLen++]=ANO_ADDR_Pilot;
//	databuf[BufLen++]=ANO_ADDR_UPPER;
//	databuf[BufLen++]=ANO_FUNC_RCdata;//   func
//	databuf[BufLen++]= 20 ;//   data len    10* int16
//	databuf[BufLen++]= BYTE1(rcdata[0]) ;//   
//	databuf[BufLen++]= BYTE0(rcdata[0]) ;//   THR
//	databuf[BufLen++]= BYTE1(rcdata[1]) ;//    
//	databuf[BufLen++]= BYTE0(rcdata[1]) ;//   YAW 
//	databuf[BufLen++]= BYTE1(rcdata[2]) ;//    
//	databuf[BufLen++]= BYTE0(rcdata[2]) ;//    ROLL
//	databuf[BufLen++]= BYTE1(rcdata[3]) ;//   
//	databuf[BufLen++]= BYTE0(rcdata[3]) ;//   PITCH
//	
//	databuf[BufLen++]= BYTE1(rcdata[4]) ;//    
//	databuf[BufLen++]= BYTE0(rcdata[4]) ;//    
//	databuf[BufLen++]= BYTE1(rcdata[5]) ;//    
//	databuf[BufLen++]= BYTE0(rcdata[5]) ;//   
//	databuf[BufLen++]= BYTE1(rcdata[6]) ;//   
//	databuf[BufLen++]= BYTE0(rcdata[6]) ;//   
//	databuf[BufLen++]= BYTE1(rcdata[7]) ;//    
//	databuf[BufLen++]= BYTE0(rcdata[7]) ;//    
//	databuf[BufLen++]= BYTE1(rcdata[8]) ;//    
//	databuf[BufLen++]= BYTE0(rcdata[8]) ;//   
//	databuf[BufLen++]= BYTE1(rcdata[9]) ;//    
//	databuf[BufLen++]= BYTE0(rcdata[9]) ;//   
//	
//	Check_SUM(databuf,BufLen++);
//	for(i=0;i<BufLen;i++)ANO_TX_BUF[i] = databuf[i];
//	ANO_TX_Enable(BufLen);
}

void ANO_Report_Motor(void)
{
//	unsigned char databuf[50];
//	unsigned char BufLen=0;
//	
//	//acc gyro mag    xyz
//	unsigned short motors[8];
//	unsigned int i;
//	
//	motors[0]=100;//
//	motors[1]=200;//
//	motors[2]=2000;
//	motors[3]=100;//
//	motors[4]=200;//
//	motors[5]=0;
//	motors[6]=100;//
//	motors[7]=200;//

//	
//	databuf[BufLen++]=ANO_Frame_Header;
//	databuf[BufLen++]=ANO_ADDR_Pilot;
//	databuf[BufLen++]=ANO_ADDR_UPPER;
//	databuf[BufLen++]=ANO_FUNC_Motor;//   func
//	databuf[BufLen++]= 16 ;//   data len    8* int16
//	databuf[BufLen++]= BYTE1(motors[0]) ;//   
//	databuf[BufLen++]= BYTE0(motors[0]) ;//   
//	databuf[BufLen++]= BYTE1(motors[1]) ;//    
//	databuf[BufLen++]= BYTE0(motors[1]) ;//    
//	databuf[BufLen++]= BYTE1(motors[2]) ;//    
//	databuf[BufLen++]= BYTE0(motors[2]) ;//    
//	databuf[BufLen++]= BYTE1(motors[3]) ;//   
//	databuf[BufLen++]= BYTE0(motors[3]) ;//   
//	
//	databuf[BufLen++]= BYTE1(motors[4]) ;//    
//	databuf[BufLen++]= BYTE0(motors[4]) ;//    
//	databuf[BufLen++]= BYTE1(motors[5]) ;//    
//	databuf[BufLen++]= BYTE0(motors[5]) ;//   
//	databuf[BufLen++]= BYTE1(motors[6]) ;//   
//	databuf[BufLen++]= BYTE0(motors[6]) ;//   
//	databuf[BufLen++]= BYTE1(motors[7]) ;//    
//	databuf[BufLen++]= BYTE0(motors[7]) ;//    
//	
//	Check_SUM(databuf,BufLen++);
//	for(i=0;i<BufLen;i++)ANO_TX_BUF[i] = databuf[i];
//	ANO_TX_Enable(BufLen);
}

void ANO_Report_Sensor2(void)
{
//	unsigned char databuf[50];
//	unsigned char BufLen=0;
//	
//	//acc gyro mag    xyz
//	unsigned short temp;
//	unsigned int alt_baro,alt_add;
//	unsigned int i;
//	
//	alt_baro=100  *100;//  0.01  cm
//	alt_add =200  *100;//   0.01  cm
//	temp=50.0   *10;

//	
//	databuf[BufLen++]=ANO_Frame_Header;
//	databuf[BufLen++]=ANO_ADDR_Pilot;
//	databuf[BufLen++]=ANO_ADDR_UPPER;
//	databuf[BufLen++]=ANO_FUNC_Sensor2;//   func
//	databuf[BufLen++]= 10 ;//   data len    1* int16   2*int32
//	databuf[BufLen++]= BYTE3(alt_baro) ;//   
//	databuf[BufLen++]= BYTE2(alt_baro) ;//   
//	databuf[BufLen++]= BYTE1(alt_baro) ;//    
//	databuf[BufLen++]= BYTE0(alt_baro) ;//   
//	
//	databuf[BufLen++]= BYTE3(alt_add) ;//    
//	databuf[BufLen++]= BYTE2(alt_add) ;//    
//	databuf[BufLen++]= BYTE1(alt_add) ;//   
//	databuf[BufLen++]= BYTE0(alt_add) ;//   
//	
//	databuf[BufLen++]= BYTE1(temp) ;//    
//	databuf[BufLen++]= BYTE0(temp) ;//    
//	
//	Check_SUM(databuf,BufLen++);
//	for(i=0;i<BufLen;i++)ANO_TX_BUF[i] = databuf[i];
//	ANO_TX_Enable(BufLen);
}

void ANO_Report_UserData1(void)//   10*short     16bits signed    -32767~+32768
{
	unsigned char databuf[50];
	unsigned char BufLen=0;

	float senddata[10];
	unsigned int i;
	

	senddata[0] = Ctrler.gyrozPID.FB;
	senddata[1] = OpenMVData.stm32_flag.stm_to_distance;
	senddata[2] = OpenMVData.openmv_top.yaw;
	

	senddata[3] =	OpenMVData.openmv_top.x;
  senddata[4] = OpenMVData.BlobData.y;
	senddata[5] = Ctrler.Z_posPID.FB;
	senddata[6] = Ctrler.Z_posPID.Des;

	senddata[7] = Ctrler.yawPID.FB;
	senddata[8] = Ctrler.locxsPID.Des;
	senddata[9] = Ctrler.gyrozPID.Des;

	databuf[BufLen++]= BYTE0(senddata[0]) ;//   
	databuf[BufLen++]= BYTE1(senddata[0]) ;//   
	databuf[BufLen++]= BYTE2(senddata[0]) ;//    
	databuf[BufLen++]= BYTE3(senddata[0]) ;// 
	
	databuf[BufLen++]= BYTE0(senddata[1]) ;//    
	databuf[BufLen++]= BYTE1(senddata[1]) ;//    
	databuf[BufLen++]= BYTE2(senddata[1]) ;//   
	databuf[BufLen++]= BYTE3(senddata[1]) ;//  
	
	databuf[BufLen++]= BYTE0(senddata[2]) ;//    
	databuf[BufLen++]= BYTE1(senddata[2]) ;//    
	databuf[BufLen++]= BYTE2(senddata[2]) ;//    
	databuf[BufLen++]= BYTE3(senddata[2]) ;//   
	
	databuf[BufLen++]= BYTE0(senddata[3]) ;//   
	databuf[BufLen++]= BYTE1(senddata[3]) ;//   
	databuf[BufLen++]= BYTE2(senddata[3]) ;//    
	databuf[BufLen++]= BYTE3(senddata[3]) ;//  
	
	databuf[BufLen++]= BYTE0(senddata[4]) ;//   
	databuf[BufLen++]= BYTE1(senddata[4]) ;//   
	databuf[BufLen++]= BYTE2(senddata[4]) ;//    
	databuf[BufLen++]= BYTE3(senddata[4]) ;//  
	
	databuf[BufLen++]= BYTE0(senddata[5]) ;//   
	databuf[BufLen++]= BYTE1(senddata[5]) ;//   
	databuf[BufLen++]= BYTE2(senddata[5]) ;//    
	databuf[BufLen++]= BYTE3(senddata[5]) ;//  
	
	databuf[BufLen++]= BYTE0(senddata[6]) ;//   
	databuf[BufLen++]= BYTE1(senddata[6]) ;//   
	databuf[BufLen++]= BYTE2(senddata[6]) ;//    
	databuf[BufLen++]= BYTE3(senddata[6]) ;//  
	
	databuf[BufLen++]= BYTE0(senddata[7]) ;//   
	databuf[BufLen++]= BYTE1(senddata[7]) ;//   
	databuf[BufLen++]= BYTE2(senddata[7]) ;//    
	databuf[BufLen++]= BYTE3(senddata[7]) ;//  
	
	databuf[BufLen++]= BYTE0(senddata[8]) ;//   
	databuf[BufLen++]= BYTE1(senddata[8]) ;//   
	databuf[BufLen++]= BYTE2(senddata[8]) ;//    
	databuf[BufLen++]= BYTE3(senddata[8]) ;// 
	
	databuf[BufLen++]= BYTE0(senddata[9]) ;//   
	databuf[BufLen++]= BYTE1(senddata[9]) ;//   
	databuf[BufLen++]= BYTE2(senddata[9]) ;//    
	databuf[BufLen++]= BYTE3(senddata[9]) ;// 
	
	databuf[BufLen++]=  0x00 ;//    
	databuf[BufLen++]=  0x00 ;//  
	databuf[BufLen++]=  0x80 ;//    
	databuf[BufLen++]=  0x7f ;// 
	
//	Check_SUM(databuf,BufLen++);
	for(i=0;i<BufLen;i++)ANO_TX_BUF[i] = databuf[i];
	ANO_TX_Enable(BufLen);
}

void Check_SUM(unsigned char* databuf,unsigned char BufLen)
{
	int i=0;
	unsigned char sum=0;
	for(i=0;i<BufLen;i++)
	{
		sum+=databuf[i];
	}
	databuf[BufLen]=sum;
}

//  AA AF 05 E0 0B E1 00 00 00 00 00 00 00 00 00 00 2A      cmd
//AA 05 AF E0 0B E1 00 00 00 00 00 00 00 00 00 00 2A AA 05 AF E1 06 00 00 20 00 00 00 65   send pid
void ANO_CMD_GetOneByte(uint8_t data)
{
	static unsigned char _data_len = 0;
	static unsigned char state = 0;
	static unsigned char  _datatemp[50];
	static unsigned char frame_type;
	static unsigned char _data_cnt;
	if(state==0&&data==0xAA)//header
	{
		state=1;
		_datatemp[0]=data;
	}
	else if(state==1&&data==0xAF)	//源地址
	{
		state=2;
		_datatemp[1]=data;
	}
	else if(state==2 &&data== 0x05)			//目的地址   05 pilot
	{
		state=3;
		_datatemp[2]=data;
	}
	else if(state==3 &&data== 0xE0)			//功能字 E0   CMD
	{
		state = 4;
		frame_type = Frame_CMD;
		_datatemp[3]=data;
	}
	else if(state==3 &&data== 0xE1)			//功能字   E1  PARAM
	{
		state = 4;
		frame_type = Frame_Param;
		_datatemp[3]=data;
	}
	else if(state==4)			//长度
	{
		state = 5;
		_datatemp[4]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==5&&_data_len>0)
	{
		_data_len--;
		_datatemp[5+_data_cnt++]=data;
		if(_data_len==0)
			state = 6;
	}
	else if(state==6)
	{
		state = 0;
		_datatemp[5+_data_cnt]=data;
		ANO_data_Decode(_datatemp,_data_cnt+6,frame_type);//
	}
	else
		state = 0;
}
/********
优秀！需要校验的部分单独拿出来，进入函数时创建局部变量sum并初始化为0,避免了一个sum归零的bug
********/

//  AA AF 05 E0 0B E1 00 00 00 00 00 00 00 00 00 00 2A      cmd
//AA 05 AF E0 0B E1 00 00 00 00 00 00 00 00 00 00 2A AA 05 AF E1 06 00 00 20 00 00 00 65   send pid
void ANO_data_Decode(unsigned char * pbuf,unsigned char buflen,unsigned char _frame_type)
{
	unsigned char sum=0;
	unsigned short read_PID_num,write_PID_num;
	int write_PID_Param;
	int i;
	if(_frame_type == Frame_CMD)
	{
		if(pbuf[4]==0x0B && pbuf[5]==0xE1)
		{
			for(i=0;i<buflen-1;i++)sum+=pbuf[i];
			if(sum!=pbuf[buflen-1])return;
			read_PID_num = pbuf[6]<<8 | pbuf[7];
			ANO_Param_Back(read_PID_num,pbuf,buflen);
		}
	}
	else if(_frame_type == Frame_Param)
	{
		if(pbuf[4]==0x06)
		{
			for(i=0;i<buflen-1;i++)sum+=pbuf[i];
			if(sum!=pbuf[buflen-1])return;
			write_PID_num = pbuf[5]<<8 | pbuf[6];
			write_PID_Param = pbuf[7]<<24 | pbuf[8]<<16 | pbuf[9]<<8 | pbuf[10];
			ChangePID(write_PID_num,write_PID_Param);
			ANO_Param_Back(write_PID_num,pbuf,buflen);
		}
	}
}


void ChangePID(unsigned short num,int param)
{
	if(num>=60)return;
	ANO_PID_Table[num] = param /1000.0f;
}

unsigned char CMD_Param_Back_BUF[100];
void ANO_Param_Back(unsigned short num,unsigned char* pbuf,unsigned char buflen)
{
	int i;
	unsigned char bufpos;
	int Param;
	CMD_Param_Back_BUF[0]=0xAA;
	CMD_Param_Back_BUF[1]=pbuf[2];
	CMD_Param_Back_BUF[2]=pbuf[1];
	for(i=3;i<buflen;i++)CMD_Param_Back_BUF[i]=pbuf[i];
	bufpos = buflen;
	
	if(num<=60)
		Param = ANO_PID_Table[num] *1000.0f;
	else  Param=0;
	
	CMD_Param_Back_BUF[bufpos++]=ANO_Frame_Header;
	CMD_Param_Back_BUF[bufpos++]=ANO_ADDR_Pilot;
	CMD_Param_Back_BUF[bufpos++]=ANO_ADDR_UPPER;
	CMD_Param_Back_BUF[bufpos++]=ANO_FUNC_Param;
	CMD_Param_Back_BUF[bufpos++]=6;
	CMD_Param_Back_BUF[bufpos++]= BYTE1(num) ;//   
	CMD_Param_Back_BUF[bufpos++]= BYTE0(num) ;// 
	CMD_Param_Back_BUF[bufpos++]= BYTE3(Param) ;//   ;
	CMD_Param_Back_BUF[bufpos++]= BYTE2(Param) ;//   ;
	CMD_Param_Back_BUF[bufpos++]= BYTE1(Param) ;//   ;
	CMD_Param_Back_BUF[bufpos++]= BYTE0(Param) ;//   ;
	
	Check_SUM(CMD_Param_Back_BUF+buflen,11);
	CMD_Param_Back_BUF[99]=bufpos+1;
	Need_to_Param_Back=1;
}

void ANO_Report_Param_Back(void)
{
//	int i;
//	for(i=0;i<CMD_Param_Back_BUF[99];i++)ANO_TX_BUF[i] = CMD_Param_Back_BUF[i];
//	ANO_TX_Enable(CMD_Param_Back_BUF[99]);
}
