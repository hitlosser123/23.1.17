#include "x_wdp.h"
/******** 0xAA 0xAA count numberofdata data sum ********/

X_WDP_TypeDef WDP_T;
X_WDP_TypeDef WDP_R;
void x_wdp_encode(X_WDP_TypeDef* pWDP)
{
	unsigned char* p;unsigned char i;
	pWDP->buf_length=0;
	pWDP->cnt++;
	pWDP->pbuf[pWDP->buf_length++]=0xAA;
	pWDP->pbuf[pWDP->buf_length++]=0xAA;
	p=(unsigned char*)&pWDP->cnt;
	pWDP->pbuf[pWDP->buf_length++]=*p++;
	pWDP->pbuf[pWDP->buf_length++]=*p++;
	pWDP->pbuf[pWDP->buf_length++]=*p++;
	pWDP->pbuf[pWDP->buf_length++]=*p++;
	p=(unsigned char*)&pWDP->number_of_data;
	pWDP->pbuf[pWDP->buf_length++]=*p++;
	pWDP->pbuf[pWDP->buf_length++]=*p++;
	p=(unsigned char*)&pWDP->pdata[0];
	for(i=0;i<pWDP->number_of_data;i++)
	{
		pWDP->pbuf[pWDP->buf_length++]=*p++;
		pWDP->sum+=pWDP->pbuf[pWDP->buf_length-1];
		pWDP->pbuf[pWDP->buf_length++]=*p++;
		pWDP->sum+=pWDP->pbuf[pWDP->buf_length-1];
	}
	pWDP->pbuf[pWDP->buf_length++]=pWDP->sum;
	pWDP->encode_state=1;
}

void x_wdp_decode(X_WDP_TypeDef* pWDP)
{
	unsigned int tempcnt;
	unsigned short tempnumberofdata;
	unsigned char i;
	if(pWDP->pbuf[0]!=0xAA){pWDP->decode_state=0;return;}
	if(pWDP->pbuf[1]!=0xAA){pWDP->decode_state=0;return;}
	pWDP->sum=0;
	tempcnt=*(unsigned int*)&pWDP->pbuf[2];
	tempnumberofdata=*(unsigned short*)&pWDP->pbuf[6];
	if(tempnumberofdata*2+9!=pWDP->buf_length)
	{
		pWDP->decode_state=Lost_Data;
		return;
	}
	for(i=0;i<tempnumberofdata;i++)
	{
		pWDP->pdata[i]=*(short*)&pWDP->pbuf[8+i*2];
		pWDP->sum+=pWDP->pbuf[8+i*2];
		pWDP->sum+=pWDP->pbuf[9+i*2];
	}
	if(tempcnt>pWDP->cnt+1)
	{
		pWDP->decode_state=Lost_Packet;
		pWDP->cnt=tempcnt;
		return;
	}
	if(pWDP->sum!=pWDP->pbuf[pWDP->buf_length-1])
	{
		pWDP->decode_state=Sum_Error;
		return;
	}
	pWDP->decode_state=Decode_OK;
}
