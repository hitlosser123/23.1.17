#ifndef    __ANO_V65_
#define    __ANO_V65_

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)      ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


#define  ANO_ADDR_UPPER    0xAF
#define  ANO_ADDR_Pilot    0x05

#define  ANO_Frame_Header  0xAA

#define  ANO_FUNC_Version  0x00
#define  ANO_FUNC_Status   0x01
#define  ANO_FUNC_Sensor   0x02
#define  ANO_FUNC_RCdata   0x03
#define  ANO_FUNC_GPSdata  0x04
#define  ANO_FUNC_Power    0x05
#define  ANO_FUNC_Motor    0x06
#define  ANO_FUNC_Sensor2  0x07

#define  ANO_FUNC_CMD       0xE0
#define  ANO_FUNC_Param     0xE1

#define  ANO_FUNC_UserData1   0xF1
#define  ANO_FUNC_UserData2   0xF2
#define  ANO_FUNC_UserData3   0xF3
#define  ANO_FUNC_UserData4   0xF4
#define  ANO_FUNC_UserData5   0xF5
#define  ANO_FUNC_UserData6   0xF6
#define  ANO_FUNC_UserData7   0xF7
#define  ANO_FUNC_UserData8   0xF8
#define  ANO_FUNC_UserData9   0xF9
#define  ANO_FUNC_UserData10  0xFA

#define  ANO_FLY_MODE_Attitude         0x01
#define  ANO_FLY_MODE_Height_Hold      0x02
#define  ANO_FLY_MODE_Position_Hold    0x03

#define   ANO_Armed      1
#define   ANO_Disarmed   0

#define  Report_Param_Back   0x55

#define Frame_CMD       1
#define Frame_Param     2

#define   ANO_PID_Table_Num    60
#define   ANO_PID_Addr_in_EEPROM    100
extern float ANO_PID_Table[ANO_PID_Table_Num];//  0reserved   1for P 2for I   3for D
extern unsigned char Need_to_Param_Back;
void  ANO_V65_Report(unsigned char SelectFunc);

void ANO_Report_Status(void);
void ANO_Report_Sensor(void);
void ANO_Report_RCdata(void);
void ANO_Report_Motor(void);
void ANO_Report_Sensor2(void);
void ANO_Report_UserData1(void);

void Check_SUM(unsigned char* databuf,unsigned char BufLen);
void ANO_data_Decode(unsigned char * pbuf,unsigned char buflen,unsigned char _frame_type);
void ChangePID(unsigned short num,int param);
void ANO_Param_Back(unsigned short num,unsigned char* pbuf,unsigned char buflen);

void ANO_EEPROM_Write(unsigned short* WriteAddr,unsigned char* buf,unsigned short buflen);
void ANO_EEPROM_Read(unsigned short ReadAddr,unsigned char* buf,unsigned short buflen);

void ANO_Save_All_PID_Table_To_EEPROM(void);
void ANO_Read_All_EEPROM_To_PID_Table(void);

void Update_Using_PID_from_EEPROM(void);
void Save_Using_PID_to_EEPROM(void);

void ANO_Report_Param_Back(void);
void ANO_CMD_GetOneByte(unsigned char data);
unsigned short Get_EEPROM_PID_Status(void);
void PID_Init(void);

void Update_Table_From_Using_PID(void);
void Update_Using_PID_from_Table(void);

#endif
