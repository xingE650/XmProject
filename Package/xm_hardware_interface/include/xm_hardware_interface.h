#ifndef XM_HARDWARE_INTERFACE_H_
#define XM_HARDWARE_INTERFACE_H_
#include "board_controller.h"
#include "robot_abstract.h"
#include "hf_link.h"
#include "tf_2wd.h"
#include "controller_manager.h"
// the xm_hardware_interface use the same namespace --xm_hw
namespace xm_hw
{
    class XmHw
    {
    public:
        XmHw()
        {
            control_command_quality=0;   //0~100  if<30 control_command_quality is bad and Control Disable  , otherwise control enable
            robot_control_en=0;
            call_frequency=20;
                    l_filter=0.4;
        }
        void HardWareInit(void);
        void HardWareUpdate(void);
        // we should use the dividing frequency to send read command to arm_motor instead of using an extra function

    private:
      
        unsigned char robot_wheel_model;
        unsigned char control_command_quality;  // 0~100  if<30 control_command_quality is bad and Control Disable  , otherwise control enable
        float call_frequency;

        void robotDataUpdate(void);
        void chassisControl(void);
        void headControl(void);
        void clawControl(void);
        void armControl(void);
        void robotCoordCalc(void);
        void platControl(void);
    };

extern RobotAbstract my_robot;
extern HFLink hf_link_pc_node;
extern HFLink *hf_link_node_pointer;
}
#endif  



