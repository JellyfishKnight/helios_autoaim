#include "TraditionalArmorDetector.hpp"

namespace helios_cv {
    TraditionalArmorDetector::TraditionalArmorDetector(helios_autoaim::Params::Detector::ArmorDetector detector_params) {

    }
    
    void TraditionalArmorDetector::set_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr cam_info) {

    }

    bool TraditionalArmorDetector::init_detector(helios_autoaim::Params::Detector detector_param) {

    }

    helios_rs_interfaces::msg::Armors TraditionalArmorDetector::detect_targets(sensor_msgs::msg::Image::SharedPtr images) {

    }

    cv::Mat& TraditionalArmorDetector::draw_results() {

    }

    void TraditionalArmorDetector::set_params(helios_autoaim::Params::Detector detector_params) {
        
    }

} // namespace helios_cv