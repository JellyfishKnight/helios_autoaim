// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

#include "rclcpp/rclcpp.hpp"

namespace helios_cv {

class BaseDetector {
public:
    BaseDetector();

    virtual bool init_detector() = 0;

    virtual bool detect_targets() = 0;

    virtual 

    ~BaseDetector();
private:

};

} // namespace helios_cv