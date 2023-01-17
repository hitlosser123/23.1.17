#ifndef __MAG_H
#define __MAG_H

typedef struct 
{
	float X_max;
	float X_min;
	float Y_max;
	float Y_min;
	float Z_max;
	float Z_min;
	float X_offset;
	float Y_offset;
	float Z_offset;
	float MAG1_SCALE;
	float MAG2_SCALE;
}MagCelTypeDef;

void Mag_ST_Init(void);
void Mag_Adjust(void);
void Mag_Calibration (void);
void Mag_Update(void);
void Mag_Save_Adjust_Data(float xMax,float xMin,float yMax,float yMin,float zMax,float zMin);

#endif
