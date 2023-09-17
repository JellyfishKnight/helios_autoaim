// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

// auto generated by ros2 generate_parameter_library
// https://github.com/PickNikRobotics/generate_parameter_library
#include "helios_autoaim_parameters.hpp"

namespace helios_cv {

class BaseFireController {
public:
    BaseFireController() = default;

    virtual void init_fire_controller(helios_autoaim::Params::FireController fire_controller_params) = 0;

    virtual bool check_fire() = 0;

    virtual void set_params(helios_autoaim::Params::FireController fire_controller_params) = 0;

private:
};

} // namespace helios_cv