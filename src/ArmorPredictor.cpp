#include "ArmorPredictor.hpp"

namespace helios_cv {
ArmorPredictor::ArmorPredictor(helios_autoaim::Params::Predictor::ArmorPredictor predictor_params) {

}

void ArmorPredictor::set_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr cam_info) {

}

void ArmorPredictor::init_predictor(helios_autoaim::Params::Predictor predictor_param) {

}

helios_rs_interfaces::msg::Target ArmorPredictor::predict_target(helios_rs_interfaces::msg::Armors armors) {

}

void ArmorPredictor::set_params(helios_autoaim::Params::Predictor predictor_params) {

}

std::vector<double> ArmorPredictor::get_state() const {
    
}

void ArmorPredictor::update_target_type(const helios_rs_interfaces::msg::Armor& armor) {

}

double ArmorPredictor::orientation2yaw(const geometry_msgs::msg::Quaternion& orientation) {

}

void ArmorPredictor::reset_kalman() {

}

void ArmorPredictor::armor_jump(const helios_rs_interfaces::msg::Armor tracking_armor) {

}

Eigen::Vector3d ArmorPredictor::state2position(const Eigen::VectorXd& state) {

}


} // namespace helios_cv