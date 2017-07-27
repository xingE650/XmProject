#ifndef MOTOR_CONTROL_XM
#define MOTOR_CONTROL_XM

#include "board_
#include <string.h>

const int PULSE[4]={503808,330000,335000,75000}; //��е�������ؽ�ÿתһȦ��������
const int encoder = 2500;
class MOTORCONTROL 
{
	public:
		MOTORCONTROL()
	{
		memset(byte,4*sizeof(uint8_t),0);
		Flag1_CAN_Receive=0;
		Flag2_CAN_Receive=0;
		memset(arm_cmd,20*sizeof(uint8_t),0);
		memset(coord,8*sizeof(uint8_t),0);
		first_call_ =0;
		wheel_position.x=0;//x
		wheel_position.y=0;//y
		wheel_position.z=0;//jiaodu
		des_.x=0;//x
		des_.y=0;//y
		des_.z=0;//jiaodu
		wheel_position.x=0;//x
		wheel_position.y=0;//y
		wheel_position.z=0;//jiaodu
		d_length =2.0;
		last_d_ =0;
		control_value =0;
		d_ =0;

		Kalman_Q=10;
		Kalman_R=1;
		Kalman_P=0;
		Kalman_Kg=0;
	}
		short speed[2];
		void chassis_init(void);
		void wheel_control(void);
		void arm_control(void);
		void arm_read(void);
		void chassis_clear(void);
		void arm_home(void);
		uint8_t coord[8];
		uint8_t byte[4];
		uint8_t pre_byte[4];
		uint8_t Flag1_CAN_Receive;
		uint8_t Flag2_CAN_Receive;
		uint8_t arm_cmd[20];
	
	
		
	    void Kalman_filtering(uint8_t pre_byte[2],uint8_t byte[2],char flag);
		void move_to_position(void);
		short get_speed_m1(void){chassis_read_speed();return speed[0];}
		short get_speed_m2(void){chassis_read_speed();return speed[1];}
		void set_position(MSGCoord position_,MSGCoord cur_y);
		
		
		void get_cur_position(MSGCoord x){cur_position = x;}
	private:
		short pre_speed;
		float d_length;
		char first_call_;
	    float Kalman_Q;
	    float Kalman_R;
	    float Kalman_P;
	    float Kalman_Kg;
		
		void chassis_read_speed(void);
		void motor_reset(unsigned int CAN_ID);
		void motor_mode(unsigned int CAN_ID,unsigned char mode);
		void motor_clear(unsigned int CAN_ID);
		void motor_speed(unsigned int CAN_ID,short temp_velocity);
		void arm_angle(unsigned char ARM_ID,float angle);
		void motor_math_itoh(int N, uint8_t *h);
		void motor_math_htoi(uint8_t *h, int *a);
		void can_timeout(void);
		void motor_config(unsigned int CAN_ID,unsigned char temp_time);
		void arm_motor_read(unsigned int CAN_ID);
		float motor_math_abs(float x){if(x<0) x=-x;return x;}
		
		void arm_clear(void);
	/********************************function: for move to position*********************************************/
		
	
		
		MSGCoord wheel_position;
		MSGCoord des_;
		MSGCoord cur_position;
		float last_d_;
		float d_;
		uint16_t control_value;
};
extern MOTORCONTROL motorcontrol;
extern RobotAbstract my_robot;


#endif
