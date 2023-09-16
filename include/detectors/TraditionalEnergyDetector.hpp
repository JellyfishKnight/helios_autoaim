// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

#include "BaseDetector.hpp"

#include "helios_autoaim_parameters.hpp"
namespace helios_cv {

class TraditionalEnergyDetector : public BaseDetector {
public:
    TraditionalEnergyDetector(helios_autoaim::Params::Detector detector_params);

    bool init_detector() override;

    bool detect_targets() override;

    void draw_results(cv::Mat& img) override;

private:
    helios_autoaim::Params::Detector detector_params_;
};

} // namespace helios_cv