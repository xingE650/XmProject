/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: robot_wheel_config.h
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* mawenke       2015.10.1   V1.0           creat this file
* Description:
*
***********************************************************************************************************************/
#include "robot_head.h"

RobotHead robot_head;

/***********************************************************************************************************************
* Function:
*
* Scope:        public
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void RobotHead::init(void)
{
    RobotHead();
    if (servo_type == 1)
    {
        axServoInit();
        pitch_range = 30 * angletorad;
        yaw_range = 80 * angletorad;
        pitch_offset = 150 * angletorad;
        yaw_offset = 150 * angletorad;
    }
    else
    {
        board.pwmInterfaceInit(TIM12 , 1);
        pitch_range = 50 * angletorad;
        yaw_range = 70 * angletorad;
        pitch_offset =  - 5 * angletorad;
        yaw_offset = 60  * angletorad;
    }
}

/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void RobotHead::topCall(void)
{

    if(first_call==0)
    {
        first_call=1;
        if(servo_type == 1){
            //getServoConnective();
            expect_head_pitch = 0 ;
            expect_head_yaw = 0 ;
						claw_state=0;
						set_head_state();
						set_claw_state();
        }
    }
    else
    {
        if(servo_type == 0) {
            set_head_state_renew = 1;
            read_head_state_renew = 1;
        }
				if(set_claw_state_renew == 1 )
        {
            set_claw_state_renew = 0;
            set_claw_state();
        }
        if(set_head_state_renew == 1 )
        {
            set_head_state_renew = 0;
            set_head_state();
						
        }
        if(read_head_state_renew == 1 )
        {
            read_head_state_renew = 0;
            read_head_state();
        }
				 
    }
}

void RobotHead::set_claw_state(void)
{
	float angle_claw;
	
	if(servo_type == 1)
		{
        
				angle_claw=(uint16_t)((claw_state*480 + 310));
    
				axSendPosition(3, angle_claw , 0x90);
			
		}
}
/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void RobotHead::set_head_state(void)
{
			float set_pitch, set_yaw;
//    float expect_head_pitch_filter_ ,  expect_head_yaw_filter_ ;

//    if(servo_type == 1){
//        expect_head_pitch_filter =  expect_head_pitch  ;
//        expect_head_yaw_filter = expect_head_yaw ;
//    }
//    else{
//        expect_head_pitch_filter =  0.2f * expect_head_pitch  + 0.8f * expect_head_pitch_filter;
//        expect_head_yaw_filter =  0.2f * expect_head_yaw  + 0.8f * expect_head_yaw_filter;
//    }

//    if(expect_head_pitch_filter >= pitch_range) expect_head_pitch_filter = pitch_range;
//    if(expect_head_pitch_filter <= -pitch_range) expect_head_pitch_filter = -pitch_range;
//    if(expect_head_yaw_filter >= yaw_range) expect_head_yaw_filter = yaw_range;
//    if(expect_head_yaw_filter <= -yaw_range) expect_head_yaw_filter = -yaw_range;
//    expect_head_pitch_filter_  = expect_head_pitch_filter + pitch_offset;
//    expect_head_yaw_filter_  = expect_head_yaw_filter + yaw_offset;

    if(servo_type == 1){
        set_pitch=(uint16_t)((expect_head_pitch* 180 /3.14 + angle_cmd1)*1023/300);
        set_yaw=(uint16_t)(((-expect_head_yaw*180.0/3.14 + angle_cmd2)*1023)/300);
				axSendPosition(1, set_pitch , 0x90);
        axSendPosition(2, set_yaw , 0x90);
				//axSendPosition(3, set_pitch , 0x90);
    }
//    else {
//        set_pitch = 1500 - expect_head_pitch_filter_ * radtoangle * 11.111f;
//        set_yaw = 1500 + expect_head_yaw_filter_ * radtoangle * 11.111f;
//        board.setPWMValue(9 , set_pitch);
//        board.setPWMValue(10, set_yaw);
//    }
}

/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void RobotHead::read_head_state(void)
{
    short int read_pitch , read_yaw;
    if(servo_type == 1){
        read_pitch = axReadPosition(1) ;
        read_yaw = axReadPosition(2) ;
        measure_head_pitch =  angletorad * (float)read_pitch * 300 / 1024 - pitch_offset;
        measure_head_yaw = angletorad * (float)read_yaw * 300 / 1024 - yaw_offset;
    }
    else {
        read_pitch = TIM12->CCR1;
        read_yaw = TIM12->CCR2;
        measure_head_pitch =  angletorad * (float)(1500 - read_pitch)/11.111f - pitch_offset ;
        measure_head_yaw =   angletorad * (float)(read_yaw -1500)/11.111f  - yaw_offset  ;
    }
}
