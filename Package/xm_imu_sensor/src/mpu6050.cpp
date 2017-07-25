#include "mpu6050.h"

MPU6050 mpu_6050;

void MPU6050::data_deal_(void)
{
	         
	mpu_angle=(float)mpu_angle_temp/32768*180;
	
			           
}