#ifndef __PID_H
#define __PID_H	 
#include "config&param.h"   
#define abs(x) ( (x)>0?(x):-(x) )



extern CtrlerTypeDef Ctrler;

void ComputePID(PIDTypeDef *pPID);
void ComputeYawPID(PIDTypeDef *pPID);

void ComputeNLPID(PIDTypeDef *pPID);

void Clear_Structure(void);



#endif

