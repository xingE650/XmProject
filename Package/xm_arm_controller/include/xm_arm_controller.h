#ifndef XM_ARM_CONTROLLER_H_
#define XM_ARM_CONTROLLER_H_

namespace xm_arm_controller
{
    class XmArmController
    {
        XmArmController()
        {
        }
        ~XmArmController()
        {
        }
        // 具体的控制器实现就不写了^_^
        // 但是尽量把名字起的合适一点
        // 除了必须暴露的控制器接口，（在controller_manager里使用），尽量写成private类型，防止数据被莫名其妙的修改。。。
    };
}

#endif