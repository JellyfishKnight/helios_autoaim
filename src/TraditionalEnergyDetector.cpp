#include "TraditionalEnergyDetector.hpp"

namespace helios_cv {
    TraditionalEnergyDetector::TraditionalEnergyDetector(helios_autoaim::Params::Detector::EnergyDetector detector_params) {

    }

    void TraditionalEnergyDetector::set_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr cam_info) {

    }


    bool TraditionalEnergyDetector::init_detector(helios_autoaim::Params::Detector detector_param) {

    }

    helios_rs_interfaces::msg::Armors TraditionalEnergyDetector::detect_targets(sensor_msgs::msg::Image::SharedPtr images) {

    }

    void TraditionalEnergyDetector::draw_results(cv::Mat& img) {

    }

    void TraditionalEnergyDetector::set_params(helios_autoaim::Params::Detector detector_params) {

    }

} // namespace helios_cv