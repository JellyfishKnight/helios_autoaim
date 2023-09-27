// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "NetArmorDetector.hpp"


namespace helios_cv {
NetArmorDetector::NetArmorDetector(helios_autoaim::Params::Detector::ArmorDetector detector_params) {
    params_ = detector_params;
}

void NetArmorDetector::set_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr cam_info) {
    
}

bool NetArmorDetector::init_detector(helios_autoaim::Params::Detector detector_param) {

}

helios_rs_interfaces::msg::Armors NetArmorDetector::detect_targets(const cv::Mat& images) {
    
}

void NetArmorDetector::draw_results(cv::Mat& img) {
    
}

void NetArmorDetector::set_params(helios_autoaim::Params::Detector detector_params) {
    
}


} // helios_cv