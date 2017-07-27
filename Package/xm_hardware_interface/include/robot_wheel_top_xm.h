#ifndef ROBOT_WHEEL_TOP_H
#define ROBOT_WHEEL_TOP_H
#include "board_controller.h"
#include  "robot_abstract.h"
#include "robot_wheel_config_xm.h"
#include "hf_link.h"


#include "tf_2wd_xm.h"
#define TF_CLASS TF_2WD


class RobotWheel:public TF_CLASS
{
public:
    RobotWheel()
    {
        control_command_quality=0;   //0~100  if<30 control_command_quality is bad and Control Disable  , otherwise control enable
        robot_control_en=0;
        call_frequency=20;
				l_filter=0.4;
    }
    void robotWheelTopInit(void);
    void robotWheelTopCall(void);
	void armtopCall(void);

private:
    struct RobotMotor{
        float  m1;
        float  m2;
        float  m3;
        float  m4;};
    RobotMotor d_motor_len_filter_ , expect_angle_speed_;
    unsigned char robot_wheel_model;
    unsigned char control_command_quality;  // 0~100  if<30 control_command_quality is bad and Control Disable  , otherwise control enable
    unsigned char robot_control_en;         // Whether or not enable motor control
    float call_frequency;
    float l_filter ;

    void robotDataUpdate(void);
    void chassisControl(void);
    void headControl(void);
		void clawControl(void);
    void armControl(void);
    void robotCoordCalc(void);
    void remoteAnalysis(void);
		void plat_move(void);
};

extern RobotAbstract my_robot;
extern HFLink hf_link_pc_node;
extern HFLink *hf_link_node_pointer;
extern RobotWheel hands_free_robot;
extern GYPOS gypos;
#endif  // #ifndef ROBOT_WHEEL_TOP_H

