#include "EnergyPredictor.hpp"
#include <rclcpp/time.hpp>

namespace helios_cv {
EnergyPredictor::EnergyPredictor(helios_autoaim::Params::Predictor::EnergyPredictor predictor_params) {

}

void EnergyPredictor::set_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr cam_info) {

}

void EnergyPredictor::init_predictor(helios_autoaim::Params::Predictor predictor_param, tf2_ros::Buffer::SharedPtr tf_buffer) {

}

helios_rs_interfaces::msg::Target EnergyPredictor::predict_target(helios_rs_interfaces::msg::Armors armors, const rclcpp::Time& now) {

}

void EnergyPredictor::set_params(helios_autoaim::Params::Predictor predictor_params) {

}

std::vector<double> EnergyPredictor::get_state() const {
    
}

} // namespace helios_cv