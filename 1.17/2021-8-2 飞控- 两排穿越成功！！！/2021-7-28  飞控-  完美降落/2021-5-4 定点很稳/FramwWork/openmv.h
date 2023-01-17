#ifndef   __OPENMV_
#define   __OPENMV_
#include "stdbool.h"

typedef struct
{
	unsigned short x;
	unsigned short y;
	unsigned short r;
	unsigned short mag;
	
	unsigned short validX;
	unsigned short validY;
	unsigned short validr;
	unsigned short validMAG;
}OpenMVCircleTypeDef;

typedef struct
{
	 short rho;
	 short theta;
	unsigned char  flags;
}OpenMVLineTypeDef;

typedef struct
{
	 short y1;
	 short y2;
	 short theta;
}OpenMVPowerLineTypeDef;

typedef struct
{
	 short x;
	 short y;
	 short mag;
}OpenMVBlobTypeDef;

typedef struct
{
	 short x1;
	 short x2;
	 short mag;
}OpenMVStickTypeDef;

typedef struct
{
  unsigned char led_flag;
	unsigned char buzzer_flag;
	unsigned char openmv_flag;
	unsigned char test_flag1;
	unsigned char test_flag2;
	unsigned chartest_flag3;
	short stm_to_distance;
}Openmv_flag_TypeDef;

typedef struct
{
   short x;
	 short y;
	 short yaw;
	 short key;
	 short valid;
	 short x_raw;
	 short y_raw;
	 short yaw_raw;

}Openmv_openmv2_TypeDef;

typedef struct
{
	unsigned char            datatype;
	OpenMVCircleTypeDef      CircleData;
	OpenMVLineTypeDef        LineData;
	OpenMVPowerLineTypeDef   PowerLineData;
	OpenMVBlobTypeDef        BlobData;
	OpenMVStickTypeDef       StickData;
	Openmv_flag_TypeDef      stm32_flag;
	Openmv_openmv2_TypeDef   openmv_top;
}OpenMVDataTypeDef;

extern OpenMVDataTypeDef  OpenMVData;

extern unsigned short blobx,bloby,blobw,blobh;
extern short openmvLocEx,openmvLocEy;

void openmv_blobs_decode(void);
void OpenMV_Get_1_Byte(unsigned char datax);

void Get_OpenMV_Circle_Data(unsigned short* circleX,unsigned short* circleY);
unsigned short Get_OpenMV_Circle_X(void);
unsigned short Get_OpenMV_Circle_Y(void);
int Get_OpenMV_Line_RHO(void);
int Get_OpenMV_Line_Theta(void);
bool OpenMV_Blob_Flag(void);

#endif
