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

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{
	u8 retry=0;	u32 ui32Rxdata;u8 ui8Rxdata;
	while(SSIBusy(SSI0_BASE))
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SSIDataPut(SSI0_BASE, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while(SSIBusy(SSI0_BASE))
	{
		retry++;
		if(retry>200)return 0;
	}
	SSIDataGet(SSI0_BASE, &ui32Rxdata);//32bits
	ui8Rxdata = ui32Rxdata&0xFF;
	return ui8Rxdata; //返回通过SPIx最近接收的数据					    
}

//连续读 reg:要读取的寄存器地址  len:要读取的长度   buf:读取到的数据存储区
u8 SPI1_Read_Len(u8 reg,u8 len,u8 *buf,uint32_t CS_PORT, uint8_t CS_IO)
{  
	u8 i=0;
	GPIO_L(CS_PORT,CS_IO);      //使能器件   	
	SPI1_ReadWriteByte(reg|0x80);    //发送读取状态寄存器命令    
	for(i=0;i<len-1;i++)buf[i]=SPI1_ReadWriteByte(reg|0x80+i+1);  //读取一个字节  
	buf[len-1]=SPI1_ReadWriteByte(0Xff);             //读取一个字节  
	GPIO_H(CS_PORT,CS_IO);                        //取消片选    
	return 0;		
}
//写一个字节 
u8 SPI1_Write_Byte(u8 reg,u8 data,uint32_t CS_PORT, uint8_t CS_IO) 				 
{
	GPIO_L(CS_PORT,CS_IO);                //使能器件   
	SPI1_ReadWriteByte(reg);   //发送写取状态寄存器命令    
	SPI1_ReadWriteByte(data);               //写入一个字节  
	GPIO_H(CS_PORT,CS_IO);  
	return 0;
}
//读一个字节   reg:寄存器地址 
u8 SPI1_Read_Byte(u8 reg,uint32_t CS_PORT, uint8_t CS_IO)
{
	u8 byte=0;
	GPIO_L(CS_PORT,CS_IO);                     //使能器件   
	SPI1_ReadWriteByte(reg|0x80);    //发送读取状态寄存器命令    
	byte=SPI1_ReadWriteByte(0Xff);             //读取一个字节  
	GPIO_H(CS_PORT,CS_IO);												//取消片选     
	return byte;   
}

