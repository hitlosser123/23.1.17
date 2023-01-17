#ifndef __X_WDP_H
#define __X_WDP_H

typedef struct 
{
	short* pdata;
	unsigned char* pbuf;
	unsigned short buf_length;
	unsigned short number_of_data;
//	unsigned short length_T;
//	unsigned short length_R;
	unsigned int cnt;
	unsigned char decode_state;
	unsigned char encode_state;
	unsigned char sum;
}X_WDP_TypeDef;

extern X_WDP_TypeDef WDP_T;
extern X_WDP_TypeDef WDP_R;

void x_wdp_encode(X_WDP_TypeDef* pWDP);
void x_wdp_decode(X_WDP_TypeDef* pWDP);

#define Encode_Default    0
#define Encode_OK    1

#define Decode_Default    0
#define Decode_OK    1
#define Lost_Packet  2
#define Lost_Data    3
#define Sum_Error    4


#endif
