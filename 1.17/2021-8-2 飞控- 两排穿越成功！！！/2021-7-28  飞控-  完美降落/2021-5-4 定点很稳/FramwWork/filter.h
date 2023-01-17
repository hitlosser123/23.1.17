#ifndef __FILTER_H
#define __FILTER_H	 
#include "includes.h"
typedef struct
{
	float xv[3];
	float yv[3];
	float input;
	float output;
}Bw30HzLPFTypeDef;

typedef struct
{
	Bw30HzLPFTypeDef GyroxLPF;
	Bw30HzLPFTypeDef GyroyLPF;
	Bw30HzLPFTypeDef GyrozLPF;
	Bw30HzLPFTypeDef AccxLPF;
	Bw30HzLPFTypeDef AccyLPF;
	Bw30HzLPFTypeDef AcczLPF;
}FiltersTypeDef;

extern FiltersTypeDef Filters;
float filterloop(float input);
float  Butterworth50HzLPF(float input);
void Butterworth30HzLPF(Bw30HzLPFTypeDef* pLPF);

#endif
