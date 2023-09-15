// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "opencv2/core.hpp"
#include <opencv2/core/mat.hpp>
#include <vector>

namespace helios_cv {

class BaseDetector {
public:
    BaseDetector();

    virtual bool init_detector() = 0;

    virtual bool detect_targets() = 0;

    virtual void draw_results(cv::Mat& img) = 0;

    ~BaseDetector();
};

} // namespace helios_cv