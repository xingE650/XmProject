#ifndef CONTROLLER_MANAGER_H_
#define CONTROLLER_MANAGER_H_
#include <string>
#include <vector>
#include "xm_arm_controller.h"
#include "xm_platform_controller.h"
#include "xm_header_controller.h"
#include "xm_wheel_controller.h"


namespace xm_hw
{
    class ControllerManager
    {
        public:
            ControllerManager()
            {
            }
            ~ControllerManager()
            {
            }
            void controller_registration(std::vector<std::string> controller_name);
            void controller_init();
            void controller_execute(std::string controller_name,bool* command_renew,float* control_value);
        private:
            std::vector<std::string> controller_name_;
            bool check_name(std::string controller_name)
            {
                for(int8_t i=0; i<controller_name_.size();i++)
                {
                    if (controller_name == controller_name_[i])
                        return true;
                }
                return false;
            }

    };
}

#endif