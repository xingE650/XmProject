#ifndef GYPOS_H
#define GYPOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"
//#include "main_includes.h"
	
class GYPOS
{
	public:
		GYPOS()
	{
		yaw_angle=0;
		memset(GYPOS_data,4*sizeof(uint8_t),0);

		gypos_fliter=0.01;
	
		d_last_yaw_angle=0;
		can2_flag=0;
	}
		float  get_gypos(void){fliter_angle();return yaw_angle;}
		
		uint8_t GYPOS_data[4];
		uint8_t can2_flag;
	private:
		float gypos_fliter;
		void fliter_angle(void);
		float d_last_yaw_angle;
		float yaw_angle;
};

extern GYPOS gypos;

#ifdef __cplusplus
}
#endif 

#endif
