#include "optical.h"
#include "StabilizerTask.h"
#include "math.h"
#include "adrc.h"

_ano_of_st ano_of;
static uint8_t _datatemp[1000] ;


//AnoOF_GetOneByte是初级数据解析函数，串口每接收到一字节光流数据，调用本函数一次，函数参数就是串口收到的数据
//当本函数多次被调用，最终接收到完整的一帧数据后，会自动调用数据解析函数AnoOF_DataAnl
void AnoOF_GetOneByte(uint8_t data)
{
	static u8 _data_len = 0,_data_cnt = 0;
    static u8 rxstate = 0;
	

	if(rxstate==0&&data==0xAA)
	{
		rxstate=1;
		_datatemp[0]=data;
	}
	else if(rxstate==1 && (data ==0XFF))
	{
		rxstate=2;
		_datatemp[1]=data;
	}
	else if(rxstate==2)
	{
		rxstate = 3;
		_datatemp[2]=data;
	}
	else if(rxstate==3&&data<50)
	{
		rxstate = 4;
		_datatemp[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(rxstate==4&&_data_len>0)
	{
		_data_len--;
		_datatemp[4+_data_cnt++]=data;
		if(_data_len==0)
			rxstate = 5;
	}
	else if(rxstate==5)
	{
		rxstate = 6;
		_datatemp[4+_data_cnt++]=data;
	}
	else if(rxstate==6)
	{
		rxstate = 0;
		_datatemp[4+_data_cnt]=data;

		AnoOF_DataAnl(_datatemp,_data_cnt+5);//
	}
	else
	{
		rxstate = 0;
	}

}
//AnoOF_DataAnl为光流数据解析函数，可以通过本函数得到光流模块输出的各项数据
//具体数据的意义，请参照匿名光流模块使用手册，有详细的介绍


static void AnoOF_DataAnl(uint8_t *data,uint8_t len)
{
	u8 check_sum1 = 0,check_sum2 = 0;
	if(*(data+3) != (len-6))	//判断数据长度是否正确
		return;
    for(u8 i=0; i<len-2; i++)
	{
        check_sum1 += *(data+i);
		check_sum2 += check_sum1;
	}
	if((check_sum1 != *(data+len-2)) || (check_sum2 != *(data+len-1)))	//判断sum校验
        return;	
	//================================================================================
	
	if(*(data+2)==0X51)//光流信息
	{
		if(*(data+4)==0)//原始光流信息
		{
			ano_of.of0_sta 		= *(data+5);
			ano_of.of0_dx  		= *(data+6);
			ano_of.of0_dy 		= *(data+7);
			ano_of.of_quality  	= *(data+8);

		}
		else if(*(data+4)==1)//高度融合后光流信息
		{
			ano_of.of1_sta 		= *(data+5);
			ano_of.of1_dx		= *((s16*)(data+6)) ;
			ano_of.of1_dy		= *((s16*)(data+8)) ;
			ano_of.of_quality  	= *(data+10);
			//
		
			ano_of.of_update_cnt++;
		}
		else if(*(data+4)==2)//惯导融合后光流信息
		{
			ano_of.of2_sta 		= *(data+5);
			ano_of.of2_dx		= *((s16*)(data+6)) ;
			ano_of.of2_dy		= *((s16*)(data+8)) ;
			ano_of.of2_dx_fix	= *((s16*)(data+10)) ;
			ano_of.of2_dy_fix	= *((s16*)(data+12)) ;			
			ano_of.intergral_x  = *((s16*)(data+14)) ;
			ano_of.intergral_y  = *((s16*)(data+16)) ;
			ano_of.of_quality  	= *(data+18);
			//

		}
	}
	else if(*(data+2)==0X34)//高度信息
	{
		ano_of.of_alt_cm = *((u32*)(data+7)) ;
		//
		ano_of.alt_update_cnt++;
	}
	else if(*(data+2)==0X01)//惯性数据
	{
		ano_of.acc_data_x = *((s16*)(data+4)) ;
		ano_of.acc_data_y = *((s16*)(data+6)) ;
		ano_of.acc_data_z = *((s16*)(data+8)) ;
		ano_of.gyr_data_x = *((s16*)(data+10)) ;
		ano_of.gyr_data_y = *((s16*)(data+12)) ;
		ano_of.gyr_data_z = *((s16*)(data+14)) ;
		//shock_sta+16

	}
	else if(*(data+2)==0X04)//姿态信息
	{
		//四元数格式
		ano_of.quaternion[0] = (*((s16*)(data+4))) *0.0001f ;
		ano_of.quaternion[1] = (*((s16*)(data+6))) *0.0001f ;
		ano_of.quaternion[2] = (*((s16*)(data+8))) *0.0001f ;
		ano_of.quaternion[3] = (*((s16*)(data+10)))*0.0001f ;
	}
}


