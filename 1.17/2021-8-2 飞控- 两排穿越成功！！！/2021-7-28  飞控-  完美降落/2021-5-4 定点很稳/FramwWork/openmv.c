#include "openmv.h"
#include "uart5_dma.h"
#include "AutoflyTask.h"

unsigned int OpenMVFrameCnt=0,OpenMVValidCnt=0;
OpenMVDataTypeDef  OpenMVData;

#define pOpenMVCircleData    (&OpenMVData.CircleData)
#define pOpenMVLineData    (&OpenMVData.LineData)
#define pOpenMVPowerLineData    (&OpenMVData.PowerLineData)
#define pOpenMVBlobData    (&OpenMVData.BlobData)

//   AA 30 05 E1 08 00 00 00 00 00 00 00 00 C8    ����ͨ��Э��
//115200    x  y   r  mag
//   AA 30 05 E2 05 00 00 00 00 00 sum          ѭ������ͨ��Э��

// 115200   �������ݲ���Ҫ���ֱ���һ�룬ֱ�ӷ�
/*********
������        AA 30 05 E3 06 rhoH rhoL disH disL thetaH thetaL sum
          6�ֽ���Ч����  rhoH��ʾrho�ĸ�8λ  rhoL��ʾrho�ĵ�8λ  sum�ǳ�sum�������ֽڵ��ۼӺ�
					dis:�˵ľ��루��˵�ֱ����ռ���ص㣩  theta:�˺��߼н�
������        AA 30 05 E4 02 disH disL sum
            dis:����������ߺ�����ͷˮƽ���루�߶Ȳ����ɷŷɿ�Ҳ�ɷ��Ӿ���
��ɫɫ������  AA 30 05 E5 06 xH xL yH yL pixelsH pixelsL sum
							x:ɫ������   y:ɫ������    pixels:ɫ�����������������
************/
#define  CircleFlag      0xE1
#define  LineFlag        0xE2
#define  StickFlag       0xE3
#define  PowerLineFlag   0xE4
#define  BlobFlag        0xE5

unsigned char OpenMVRXStateMachine=0;
void OpenMV_Get_1_Byte(unsigned char datax)
{
	static unsigned char datalength,p,databuf[20],tempsum,datatype;
	if(OpenMVRXStateMachine==0)
	{
		if(datax==0xAA)
		{
			OpenMVFrameCnt++;
			tempsum=datax;
			OpenMVRXStateMachine++;
		}
	}
	else if(OpenMVRXStateMachine==1)
	{
		if(datax==0x30)
		{
			tempsum+=datax;
			OpenMVRXStateMachine++;
		}
		else  OpenMVRXStateMachine=0;
	}
	else if(OpenMVRXStateMachine==2)
	{
		if(datax==0x05)
		{
			tempsum+=datax;
			OpenMVRXStateMachine++;
		}
		else  OpenMVRXStateMachine=0;
	}
	else if(OpenMVRXStateMachine==3)
	{
//		if(datax==0xE1)
//		{
//			tempsum+=datax;
//			OpenMVRXStateMachine++;
//		}
//		else  OpenMVRXStateMachine=0;
			datatype=datax;
			tempsum+=datax;
			OpenMVRXStateMachine++;	
	}
	else if(OpenMVRXStateMachine==4)
	{
		datalength = datax;
		p=0;
		tempsum+=datax;
		OpenMVRXStateMachine++;
	}
	else if(OpenMVRXStateMachine==5)
	{
		databuf[p++] = datax;
		tempsum+=datax;
		if(p==datalength)OpenMVRXStateMachine++;
	}
	else if(OpenMVRXStateMachine==6)
	{
		if(tempsum==datax)
		{
			if(datatype==CircleFlag)
			{
//				OpenMVValidCnt++;
				pOpenMVCircleData->x   = databuf[0]<<8 | databuf[1];
				pOpenMVCircleData->y   = databuf[2]<<8 | databuf[3];
				pOpenMVCircleData->r   = databuf[4]<<8 | databuf[5];
				pOpenMVCircleData->mag = databuf[6]<<8 | databuf[7];
				
				if(
						 pOpenMVCircleData->x!=0   &&
							pOpenMVCircleData->y!=0   &&
							pOpenMVCircleData->r>=20   &&
							pOpenMVCircleData->mag>=1700 
				)
				{
					pOpenMVCircleData->validX = pOpenMVCircleData->x;
					pOpenMVCircleData->validY = pOpenMVCircleData->y;
				}
			}
			else if(datatype==LineFlag)
			{
//				OpenMVValidCnt++;
				pOpenMVLineData->rho   = databuf[0]<<8 | databuf[1];//   �Ҵ�   0-80
				pOpenMVLineData->theta = databuf[2]<<8 | databuf[3];//  ��  ��0   ��+
				pOpenMVLineData->flags = databuf[4];
				
				//  left   0.3m/s
				//  rho<40   forward v
				//  rho>40   backward
				//  theta+   clock   right
				//  theta-   anticlock   left
				
				//pOpenMVCircleData->r   = databuf[4]<<8 | databuf[5];
				//pOpenMVCircleData->mag = databuf[6]<<8 | databuf[7];
				
//				if(
//						 pOpenMVCircleData->x!=0   &&
//							pOpenMVCircleData->y!=0   &&
//							pOpenMVCircleData->r>=20   &&
//							pOpenMVCircleData->mag>=1700 
//				)
//				{
//					pOpenMVCircleData->validX = pOpenMVCircleData->x;
//					pOpenMVCircleData->validY = pOpenMVCircleData->y;
//				}
			}
			else if(datatype==PowerLineFlag)//����  ����ר��
			{
				pOpenMVPowerLineData->y1    = databuf[0]<<8 | databuf[1];
				pOpenMVPowerLineData->y2    = databuf[2]<<8 | databuf[3];//   w160   h120
				pOpenMVPowerLineData->theta = databuf[4]<<8 | databuf[5];//  160*120
			}
			else if(datatype==StickFlag) //����  ����ר��
			{
			}
			else if(datatype==BlobFlag) // ��ɫ���� ����ר��
			{
//				pOpenMVBlobData->x   = databuf[0]<<8 | databuf[1];
//				pOpenMVBlobData->y   = databuf[2]<<8 | databuf[3];//   w320   h240
//				pOpenMVBlobData->mag = databuf[4]<<8 | databuf[5];//  320 240
				pOpenMVBlobData->x   = databuf[0];
				pOpenMVBlobData->y   = databuf[1];//  
				pOpenMVBlobData->mag = databuf[2];//  
				
				if(pOpenMVBlobData->x >100)
					pOpenMVBlobData->x = pOpenMVBlobData->x -256;
				if(pOpenMVBlobData->y >100)
					pOpenMVBlobData->y = pOpenMVBlobData->y -256;
				if(pOpenMVBlobData->mag >100)
					pOpenMVBlobData->mag = pOpenMVBlobData->mag -256;
	
					OpenMVValidCnt++;
				
			}
			
		}
		else  OpenMVRXStateMachine=0;
	}
	else  OpenMVRXStateMachine=0;
	

}

void Get_OpenMV_Circle_Data(unsigned short* circleX,unsigned short* circleY)
{
	*circleX = pOpenMVCircleData->validX;
	*circleY = pOpenMVCircleData->validY;
}
//openmv4   x+ ǰ     y+��     x160 y120
unsigned short Get_OpenMV_Circle_X(void)
{
	return pOpenMVBlobData->x ;
}

unsigned short Get_OpenMV_Circle_Y(void)
{
	return pOpenMVBlobData->y ;
}

int Get_OpenMV_Line_RHO(void)
{
	return  pOpenMVLineData->rho;
}

int Get_OpenMV_Line_Theta(void)
{
	return pOpenMVLineData->theta;
}




bool OpenMV_Blob_Flag(void)
{
	if(pOpenMVBlobData->mag!=0)return true;
	else return false;
}






//////////////////////////////////////////////////
#define  openmvRxBuf    UART5_RX_BUF
unsigned short blobx,bloby,blobw,blobh;
short openmvLocEx,openmvLocEy;
void openmv_blobs_decode(void)
{
	if(openmvRxBuf[0]==0x55  &&  openmvRxBuf[9]==0xAA)
	{
		blobx = openmvRxBuf[1]<<8  | openmvRxBuf[2];
		bloby = openmvRxBuf[3]<<8  | openmvRxBuf[4];
		blobw = openmvRxBuf[5]<<8  | openmvRxBuf[6];
		blobh = openmvRxBuf[7]<<8  | openmvRxBuf[8];
		
		openmvLocEx=bloby-80;
		openmvLocEy=blobx-60;
	}
}
