#include "spi1.h"
#include "delay.h"
#include "x_gpio.h"

void SPI1_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);

    SSIEnable(SSI0_BASE);

    //while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0])){ }
}   

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI1_ReadWriteByte(u8 TxData)
{
	u8 retry=0;	u32 ui32Rxdata;u8 ui8Rxdata;
	while(SSIBusy(SSI0_BASE))
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SSIDataPut(SSI0_BASE, TxData); //ͨ������SPIx����һ������
	retry=0;

	while(SSIBusy(SSI0_BASE))
	{
		retry++;
		if(retry>200)return 0;
	}
	SSIDataGet(SSI0_BASE, &ui32Rxdata);//32bits
	ui8Rxdata = ui32Rxdata&0xFF;
	return ui8Rxdata; //����ͨ��SPIx������յ�����					    
}

//������ reg:Ҫ��ȡ�ļĴ�����ַ  len:Ҫ��ȡ�ĳ���   buf:��ȡ�������ݴ洢��
u8 SPI1_Read_Len(u8 reg,u8 len,u8 *buf,uint32_t CS_PORT, uint8_t CS_IO)
{  
	u8 i=0;
	GPIO_L(CS_PORT,CS_IO);      //ʹ������   	
	SPI1_ReadWriteByte(reg|0x80);    //���Ͷ�ȡ״̬�Ĵ�������    
	for(i=0;i<len-1;i++)buf[i]=SPI1_ReadWriteByte(reg|0x80+i+1);  //��ȡһ���ֽ�  
	buf[len-1]=SPI1_ReadWriteByte(0Xff);             //��ȡһ���ֽ�  
	GPIO_H(CS_PORT,CS_IO);                        //ȡ��Ƭѡ    
	return 0;		
}
//дһ���ֽ� 
u8 SPI1_Write_Byte(u8 reg,u8 data,uint32_t CS_PORT, uint8_t CS_IO) 				 
{
	GPIO_L(CS_PORT,CS_IO);                //ʹ������   
	SPI1_ReadWriteByte(reg);   //����дȡ״̬�Ĵ�������    
	SPI1_ReadWriteByte(data);               //д��һ���ֽ�  
	GPIO_H(CS_PORT,CS_IO);  
	return 0;
}
//��һ���ֽ�   reg:�Ĵ�����ַ 
u8 SPI1_Read_Byte(u8 reg,uint32_t CS_PORT, uint8_t CS_IO)
{
	u8 byte=0;
	GPIO_L(CS_PORT,CS_IO);                     //ʹ������   
	SPI1_ReadWriteByte(reg|0x80);    //���Ͷ�ȡ״̬�Ĵ�������    
	byte=SPI1_ReadWriteByte(0Xff);             //��ȡһ���ֽ�  
	GPIO_H(CS_PORT,CS_IO);												//ȡ��Ƭѡ     
	return byte;   
}

