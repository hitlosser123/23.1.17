#include "us_100.h"
#include "uart6_dma.h"

unsigned short US_100_Distance;
unsigned char  US_100_RX_BUF[2];
unsigned char  US_100_State_Machine=0;
//  63HZ    15.9ms     9600baud  1byte 1ms
void US_100_Update(void)
{
	static unsigned short temp_distance;
	static unsigned int Byte0_SysTickCnt,Byte_DT;
	
	if(US_100_State_Machine==0)//���յ�һ���ֽ�
	{
		if(UARTCharsAvail(UART6_BASE))
		{
			US_100_RX_BUF[0]=UARTCharGetNonBlocking(UART6_BASE);
			Byte0_SysTickCnt = SysTickCnt;
			US_100_State_Machine=1;
		}
	}
	
	if(US_100_State_Machine==1)//���յڶ����ֽ�
	{
		if(UARTCharsAvail(UART6_BASE))
		{
			US_100_RX_BUF[1]=UARTCharGetNonBlocking(UART6_BASE);
			Byte_DT = SysTickCnt - Byte0_SysTickCnt;
			if(Byte_DT<=10)//��һ�ε�����
			{
				US_100_State_Machine=2;
			}
			else//���ݶϿ���
			{
				US_100_RX_BUF[0] = US_100_RX_BUF[1];
				Byte0_SysTickCnt = SysTickCnt;
				US_100_State_Machine=1;
			}
		}
	}
	
	if(US_100_State_Machine==2)//���ݽ���
	{
		temp_distance = US_100_RX_BUF[0]<<8|US_100_RX_BUF[1];//  mm
		if(temp_distance >= 100 && temp_distance <=3000)US_100_Distance=temp_distance;
		US_100_State_Machine=0;
	}
	
	
	UARTCharPutNonBlocking(UART6_BASE,0X55);//��������
}
