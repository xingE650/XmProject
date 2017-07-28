/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: robot_wheel_top.c
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license. 
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* mawenke       2015.10.1    V1.0          creat this file
* mawenke       2015.7.1     V2.0          update this file
* Description:  
***********************************************************************************************************************/
#include "xm_hardware_interface.h"

namespace xm_hw
{
    
    static const float degree_to_radian = 0.017453f;
    static const float radian_to_degree = 57.2958f;

    HFLink hf_link_pc_node(0x11,0x01,&my_robot);
    HFLink *hf_link_node_pointer=&hf_link_pc_node;
    RobotAbstract xm_robot;
    RobotWheel hands_free_robot;

    /***********************************************************************************************************************
    * Function:     void RobotWheel::robotWheelTopInit(void)
    *
    * Scope:        public
    *
    * Description:  robot init
    *
    * Arguments:
    *
    * Return:
    *
    * Cpu_Time:  
    *
    * History:
    ***********************************************************************************************************************/
    void RobotWheel::robotWheelTopInit(void)
    {
        
        robot_wheel_model = ROBOT_WHEEL_MODEL;
        
        set_robot_wheel_radius(my_robot.robot_parameters.robot_wheel_radius);
        set_robot_body_radius(my_robot.robot_parameters.robot_body_radius);
        l_filter = my_robot.robot_parameters.speed_low_filter;
            motorcontrol.chassis_init();
            motorcontrol.arm_home();
    
    }

    /***********************************************************************************************************************
    * Function:     void RobotWheel::robotWheelTopCall(void)   
    *
    * Scope:        public
    *
    * Description:  robot control interface, you must call it in a frequency , generaly  can set
    *               be a half of pid frequency
    *
    * Arguments:
    *
    * Return:
    *
    * Cpu_Time:  
    *
    * History:
    ***********************************************************************************************************************/
    void RobotWheel::robotWheelTopCall(void)
    {
    //    remoteAnalysis();

                robotDataUpdate();    //�ٶȸ���
                robotCoordCalc();    // for robot localization  ���̸���
                chassisControl();    // control your robotic chassis
                
                headControl();       // control your robotic head
                armControl();        // control your robotic arm
                clawControl();			// control your robotic claw
                plat_move();
    }

    void RobotWheel::armtopCall(void)
    {
        motorcontrol.arm_read();
    }


    /***********************************************************************************************************************
    * Function:     void RobotWheel::robotDataUpdate(void)
    *
    * Scope:        private
    *
    * Description:  update the robot RobotAbstract ,only need a low call frequency , <=20HZ is recommended
    *
    *
    * Arguments:
    *
    * Return:
    *
    * Cpu_Time:
    *
    * History:
    ***********************************************************************************************************************/
    void RobotWheel::robotDataUpdate(void)
    {
        //chassis
    //    my_robot.measure_global_speed.x = 0;
    //    my_robot.measure_global_speed.y = 0;
    //    my_robot.measure_global_speed.z = 0;
        my_robot.measure_robot_speed.x =0;
        my_robot.measure_robot_speed.y =0;
        my_robot.measure_robot_speed.z =0;

        my_robot.measure_motor_speed.servo1 = motorcontrol.get_speed_m1() /(2940.0)*2*3.1416;//�õ��������ٶ�
        my_robot.measure_motor_speed.servo2 =	motorcontrol.get_speed_m2() /(2940.0)*2*3.1416;
            my_robot.plat_height_back = platform.current_h/100;

        getRobotSpeed((float* )&my_robot.measure_motor_speed , (float* )&my_robot.measure_robot_speed);
        getGlobalSpeed((float* )&my_robot.measure_motor_speed , (float*)&my_robot.measure_global_speed , my_robot.measure_global_coordinate.z);
            my_robot.measure_global_coordinate.z =gypos.get_gypos()*degree_to_radian;



        if(hf_link_node_pointer->receive_package_renew[CLEAR_COORDINATE_DATA]==1)
        {
            hf_link_node_pointer->receive_package_renew[CLEAR_COORDINATE_DATA]=0;
            my_robot.measure_global_coordinate.x=0;
            my_robot.measure_global_coordinate.y=0;
            my_robot.measure_global_coordinate.z=0;
            my_robot.measure_robot_coordinate.x=0;
            my_robot.measure_robot_coordinate.y=0;
            my_robot.measure_robot_coordinate.z=0;
    
            
        }


    }

    /***********************************************************************************************************************
    * Function:     void RobotWheel::chassisControl(void)           
    *
    * Scope:        private
    *
    * Description:  control your robotic chassis
    *
    * Arguments:
    *
    * Return:
    *
    * Cpu_Time:  
    *
    * History:
    ***********************************************************************************************************************/
    void RobotWheel::chassisControl(void)    
    { 	 

    
            if( hf_link_node_pointer->receive_package_renew[SET_ROBOT_SPEED]==1 ){
        // hf_link_node_pointer->receive_package_renew[SET_ROBOT_SPEED ]=0;
            control_command_quality += 30;
            robotSpeedSet((float* )&my_robot.expect_robot_speed , (float* )&my_robot.expect_motor_speed);
        }
                
            control_command_quality=control_command_quality-(100.00f/call_frequency); //control_command_quality reduce 1 every call
        if(control_command_quality >= 100){
            control_command_quality=100;
        }
        else if(control_command_quality <= 5){
            control_command_quality = 5;
        }
        if(control_command_quality <=30 ){  //if control_command_quality is too low
            robot_control_en = 0;  //motor control disable
        }
        else if(control_command_quality >=70 ){  //if control_command_quality is too low
            robot_control_en = 1;  //motor control disable
        }

        if(robot_control_en == 0)
        {
            motorcontrol.chassis_clear();   //set motor  Speed 0

        }
        else if(robot_control_en == 1)
        {
                motorcontrol.wheel_control();

        }

    }

    /***********************************************************************************************************************
    * Function:     void RobotWheel::headControl(void)         
    *
    * Scope:        private
    *
    * Description:  control your robotic head
    *
    * Arguments:
    *
    * Return:
    *
    * Cpu_Time:  
    *
    * History:
    ***********************************************************************************************************************/
    void RobotWheel::headControl(void)
    {
    #ifdef  PAKG_SERVO
        float measure_head1_pitch , measure_head1_yaw;
        robot_head.getState(&measure_head1_pitch , &measure_head1_yaw);
        my_robot.measure_head1_state.pitch = measure_head1_pitch;
        my_robot.measure_head1_state.roll = 0;
        my_robot.measure_head1_state.yaw = measure_head1_yaw;
        my_robot.measure_head2_state.pitch = 0;
        my_robot.measure_head2_state.roll = 0;
        my_robot.measure_head2_state.yaw =0;

        if( hf_link_node_pointer->receive_package_renew[SET_HEAD_1]==1 ){
            hf_link_node_pointer->receive_package_renew[SET_HEAD_1]=0;
            robot_head.setState(my_robot.expect_head1_state.pitch , my_robot.expect_head1_state.yaw);
        }
    #endif
    }
    void RobotWheel::clawControl()
    {
    //	if( hf_link_node_pointer->receive_package_renew[CLAW_CONFIG]==1 ){
    //       hf_link_node_pointer->receive_package_renew[CLAW_CONFIG]=0;
    //       robot_head.setclaw_angle((float*)&(my_robot.claw_angle_));
    //	}
            
        if( hf_link_node_pointer->receive_package_renew[SET_CLAW]==1 ){
            hf_link_node_pointer->receive_package_renew[SET_CLAW]=0;
            robot_head.setclaw(my_robot.claw_state_);
        }
    }

    /***********************************************************************************************************************
    * Function:     void RobotWheel::armControl(void)       
    *
    * Scope:        private
    *
    * Description:  control your robotic arm
    *
    * Arguments:
    *
    * Return:
    *
    * Cpu_Time:  
    *
    * History:
    ***********************************************************************************************************************/
    void RobotWheel::armControl(void)
    {
                    //motorcontrol.arm_read();				
                    if(hf_link_node_pointer->receive_package_renew[SET_ARM_TOTAL]){
                            hf_link_node_pointer->receive_package_renew[SET_ARM_TOTAL]=0;						
                            motorcontrol.arm_control();			
                                        }
    }

    void RobotWheel::plat_move(void)
    {
        if( hf_link_node_pointer->receive_package_renew[PLAT_MOVE]==1){
            hf_link_node_pointer->receive_package_renew[PLAT_MOVE]=0;
            platform.set_height(my_robot.plat_height);
                }		
    }
    /***********************************************************************************************************************
    * Function:     void RobotWheel::robotCoordCalc(void)
    *
    * Scope:        private
    *
    * Description:  calculating coordinates, this is a  common methods  for robot localization
    *               
    *
    * Arguments:
    *
    * Return:
    *
    * Cpu_Time:  
    *
    * History:
    ***********************************************************************************************************************/
    void RobotWheel::robotCoordCalc(void)
    {
            
            d_motor_len_filter_.m1 = 0.02* my_robot.measure_motor_speed.servo1*get_robot_wheel_radius();//0.045* motorcontrol.get_distance_m1() * degree_to_radian * get_robot_wheel_radius();//measure wheel distance degree:m
        d_motor_len_filter_.m2 = 0.02* my_robot.measure_motor_speed.servo2*get_robot_wheel_radius();//0.045* motorcontrol.get_distance_m2() * degree_to_radian * get_robot_wheel_radius();



        //need time stm32F1 330us  stm32F4+NOFPU 64~80us   stm32F4+FPU 8~16us
        getGlobalCoordinate( (float* )&d_motor_len_filter_ , (float* )&my_robot.measure_global_coordinate);
        //need time stm32F1 20us  stm32F4+FPU 0~2us
        getRobotCoordinate( (float* )&d_motor_len_filter_ , (float* )&my_robot.measure_robot_coordinate);
        
    }

    void RobotWheel::remoteAnalysis(void)
    {
    #ifdef PACK_SBUS_PPM
        if ( sbus.sbus_state == 1)
        {
            if( sbus.sbus_channel[5] >= 1000 )
            {
                my_robot.expect_robot_speed.x = (-(sbus.sbus_channel[1] - 992)*0.001);
                my_robot.expect_robot_speed.y = (-(sbus.sbus_channel[0] - 992)*0.001);
                my_robot.expect_robot_speed.z = (-(sbus.sbus_channel[3] - 992)*0.001)*3;
                hf_link_node_pointer->receive_package_renew[SET_ROBOT_SPEED]=1;
            }
            else if( sbus.sbus_channel[5] <= 200 )
            {
                my_robot.expect_head1_state.pitch =(sbus.sbus_channel[1] - 992)*0.05;
                my_robot.expect_head1_state.yaw = -(sbus.sbus_channel[0] - 992)*0.05;
                hf_link_node_pointer->receive_package_renew[SET_HEAD_1]=1;
            }
            else
            {

            }
        }
    #endif
    }
}//use namespace xm_hw
