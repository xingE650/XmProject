#ifndef ROBOT_HEAD_H
#define ROBOT_HEAD_H

#include "servo_config.h"
#include "servo_digital.h"
//#include <string>
class RobotHead:public ServoDigital
{
public:
    RobotHead(){
        servo_type = 1;
        first_call=0;
        set_head_state_renew=0;
        read_head_state_renew=0;
        radtoangle = 57.295779513 ;
        angletorad = 0.01745329252;
        expect_head_pitch=0;
        expect_head_yaw=0;
        measure_head_pitch=0;
        measure_head_yaw=0;
        expect_head_pitch_filter = 0;
        expect_head_yaw_filter =0;
				claw_state=0;
				set_claw_state_renew=0;
				angle_cmd1=90;
				angle_cmd2=185.337;
				claw_angle[0]=0;
				claw_angle[1]=1.70;
    }
    void init(void);
    void topCall(void);

    void setState(float pitch , float yaw){ //radian
        set_head_state_renew = 1;
        expect_head_pitch =  pitch;
        expect_head_yaw = yaw;
    }
    void getState(float* pitch , float* yaw){ //radian
        read_head_state_renew = 1;
        *pitch = measure_head_pitch ;
        *yaw = measure_head_yaw;
    }
	void setclaw(uint8_t state){
			set_claw_state_renew = 1;
			claw_state=state;
		}
//			void setclaw_angle(float *angle){
//			set_claw_state_renew = 1;
//			memcpy(claw_angle,angle,sizeof(claw_angle));
//		}

private:
    void set_head_state(void);
    void read_head_state(void);
		void set_claw_state(void);

    unsigned char  servo_type;  // 1 :  digital servo  0: analog servo
    unsigned char first_call;
    unsigned char set_head_state_renew , read_head_state_renew,set_claw_state_renew;
		unsigned char claw_state;
    float radtoangle , angletorad ;    //radian to angle
    float expect_head_pitch , expect_head_yaw ,  expect_head_pitch_filter , expect_head_yaw_filter; //radian
    float measure_head_pitch , measure_head_yaw; //radian
    float pitch_offset , pitch_range , yaw_offset , yaw_range; //radian
		float angle_cmd1,angle_cmd2;
		float claw_angle[2];
};

extern RobotHead robot_head;

#endif // #ifndef ROBOT_HEAD_H






