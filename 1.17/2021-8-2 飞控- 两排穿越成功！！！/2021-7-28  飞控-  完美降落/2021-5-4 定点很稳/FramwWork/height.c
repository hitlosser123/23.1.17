#include "height.h"
#include "tf_mini_plus.h"
#include "SINS.h"
#include "StabilizerTask.h"
#include "spl06.h"
#include "optical.h"
#include "pid.h"
#include "openmv.h"

#define abs(x) ( (x)>0?(x):-(x) )
int flag_car = 0;
float last_height = 0.08f;
int i = 0 ;
void Height_Update(void)
{
	spl06_update();				//Update DT=20ms
	pressure = spl06_get_pressure();
	temperature = spl06_get_temperature();
	baro_alt = spl06_get_altitude(pressure,groud_press,groud_temp);		//alt now		
	
	data.SPL06Data.raw_h = baro_alt;
	data.SPL06Data.f1_h = data.SPL06Data.f1_h * 0.9f + data.SPL06Data.raw_h * 0.1f ;
	data.SPL06Data.f2_h = data.SPL06Data.f2_h * 0.9f + data.SPL06Data.f1_h * 0.1f ;
	data.SPL06Data.v_h = (data.SPL06Data.f2_h - data.SPL06Data.last_h )/CTRL_DT*1000.0f;
	data.SPL06Data.last_h = data.SPL06Data.f2_h;
	data.SPL06Data.f_v_h = data.SPL06Data.f_v_h *0.9f + data.SPL06Data.v_h*0.1f;
	data.SPL06Data.f2_v_h = data.SPL06Data.f2_v_h *0.95f + data.SPL06Data.f_v_h*0.05f;
	
	Strapdown_INS_High(  baro_alt *100.0f);//输入  厘米!!!!!!!!!!!!!!!
	
//	Strapdown_INS_Horizontal(data.OpticalData.loc_x,data.OpticalData.loc_y);
//	Strapdown_INS_Horizontal(data.OpticalData.temp_locx,data.OpticalData.temp_locy);
	Strapdown_INS_Horizontal(data.OpticalLC302.tempLocx,data.OpticalLC302.tempLocy);
	
//			Ctrler.Z_posPID.FB = data.TofData.f2_h;
//		Ctrler.Z_ratePID.FB = data.TofData.f2_v_h ;

	ano_of.DISTANCE_X = ano_of.DISTANCE_X+ano_of.of2_dx_fix*0.005f;
	ano_of.DISTANCE_Y = ano_of.DISTANCE_Y+ano_of.of2_dy_fix*0.005f;
	

//	 data.TofData.raw_h = TF_Mini_Plus_Distance/100.0f *Cos_Roll*Cos_Pitch;  //油门值为3220
	 
	 if( ano_of.of_alt_cm>0.01 && ano_of.of_alt_cm<180)
	 {
		 data.TofData.raw_h = ano_of.of_alt_cm*0.01 *Cos_Roll*Cos_Pitch;  //油门值为3170
	 }

//		 
//	if(data.TofData.last_high == 1) //标志位
//	{	
//	//防止高度突变代码
////		if(last_height<1.0f && data.TofData.raw_h<1.0f && (abs(data.TofData.raw_h -last_height)>0.05f &&abs(data.TofData.raw_h -last_height)<0.90f)) //发生了高度突变  进入小车
////	 {
////     data.TofData.raw_h  =  last_height;
//		if(data.TofData.raw_h<0.8f)
//		{
//		 Ctrler.Z_posPID.Kp = 0.1;
//	  }
//	 else
//	 { 
//		data.TofData.raw_h  =  data.TofData.raw_h;
//		  Ctrler.Z_posPID.Kp = 0.5;
//	 }
// 
//	}	
	
    if(data.TofData.raw_h>0.01f&&data.TofData.raw_h<1.5f)
		{
  	last_height =data.TofData.raw_h;
		}
		 
	
	
   if(data.TofData.raw_h>0.01f&&data.TofData.raw_h<1.5f)
	 data.TofData.f2_h =data.TofData.raw_h;
	 else
	 data.TofData.f2_h =data.TofData.f2_h;
	 
	 
	data.TofData.v_h = (data.TofData.f2_h - data.TofData.last_h )/CTRL_DT*1000.0f;
	data.TofData.last_h = data.TofData.f2_h;
	data.TofData.f2_v_h = data.TofData.f2_v_h *0.9f + data.TofData.v_h*0.1f;
	
}
