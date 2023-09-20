#include "TraditionalEnergyDetector.hpp"

namespace helios_cv {
TraditionalEnergyDetector::TraditionalEnergyDetector(helios_autoaim::Params::Detector::EnergyDetector detector_params) {
    detector_params_ = detector_params;
}

void TraditionalEnergyDetector::set_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr cam_info) {
    cam_info_ = cam_info;
    cam_center_ = cv::Point2f(cam_info_->k[2], cam_info_->k[5]);
    pnp_solver_ = std::make_shared<PnPSolver>(cam_info->k, cam_info->d);
}


bool TraditionalEnergyDetector::init_detector(helios_autoaim::Params::Detector detector_param) {

}

helios_rs_interfaces::msg::Armors TraditionalEnergyDetector::detect_targets(const cv::Mat& images) {

}

void TraditionalEnergyDetector::draw_results(cv::Mat& img) {

}

void TraditionalEnergyDetector::set_params(helios_autoaim::Params::Detector detector_params) {

}


cv::Mat TraditionalEnergyDetector::preprocess(cv::Mat src, bool isred) {

}

bool TraditionalEnergyDetector::find_target_flow(cv::Mat src, std::vector<std::vector<cv::Point2f>> &contours) {

}

bool TraditionalEnergyDetector::find_target_R(std::vector<std::vector<cv::Point2f>> &contours) {

}

void TraditionalEnergyDetector::setTransform(cv::Point2f p[], cv::Point2f d[]) {

}


cv::Mat TraditionalEnergyDetector::get_Roi(cv::Mat src){

} 

cv::Point2f TraditionalEnergyDetector::R_possible() {

}

void TraditionalEnergyDetector::getPts(cv::RotatedRect &armor_fin) {

}
void TraditionalEnergyDetector::setPoint(cv::RotatedRect &armor_fin, cv::Point2f &circle_center_point){

}
} // namespace helios_cv