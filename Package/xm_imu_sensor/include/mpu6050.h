#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"

	
#ifdef __cplusplus
}
#endif 	
class MPU6050
{
	public:
		MPU6050()
	{
		data_len =0;
		memset(mpu_data,2*sizeof(uint8_t),0);
		mpu_6050_state=0;
		mpu_angle_temp=0;
		mpu_angle=0;
}
	uint8_t data_len;
	uint8_t mpu_data[256];
	uint8_t mpu_6050_state;
	uint16_t mpu_angle_temp;
	float get_mpu_data(void){data_deal_();return mpu_angle;}
	private:
		float mpu_angle;
		void data_deal_(void);

};

extern MPU6050 mpu_6050;



#endif
