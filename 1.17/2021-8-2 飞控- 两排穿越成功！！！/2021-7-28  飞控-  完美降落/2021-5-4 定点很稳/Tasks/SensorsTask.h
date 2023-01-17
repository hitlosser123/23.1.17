#ifndef __SENSORSTASK_H
#define __SENSORSTASK_H	 
#include "includes.h"

void sensorsTask(void *pvParameters);

extern 	float groud_temp,groud_press;		//
extern	float pressure,temperature;
extern	float baro_alt;								//unit m
extern	float mag[3];								//¥≈¡¶

#endif
