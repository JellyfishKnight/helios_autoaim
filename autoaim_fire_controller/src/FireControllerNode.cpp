// created by liuhan on 2023/11/4
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#include "FireControllerNode.hpp"
#include <angles/angles.h>
#include <cmath>
#include <rclcpp/logging.hpp>


namespace helios_cv {

using namespace std::chrono_literals;

FireController::FireController(const rclcpp::NodeOptions& options) : 
    rclcpp::Node("fire_controller", options) {
    // create param
    try {
        param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
        params_ = param_listener_->get_params();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to start param listener: %s", e.what());
    }
    // create utilities
    bullet_solver_ = std::make_shared<BulletSolver>();
    target_solver_ = std::make_shared<TargetSolver>();
    if (!bullet_solver_ || !target_solver_) {
        RCLCPP_ERROR(logger_, "Failed to create utilities, try again");
        bullet_solver_ = std::make_shared<BulletSolver>();
        target_solver_ = std::make_shared<TargetSolver>();
    }
    bullet_solver_->update_bullet_params(BulletParams{
        params_.bullet_solver.bullet_speed,
        params_.bullet_solver.bullet_mass,
        params_.bullet_solver.air_coeff
    });
    // create publisher and subscription
    gimbal_pub_ = this->create_publisher<helios_control_interfaces::msg::GimbalCmd>("/gimbal_cmd_angle", 10);
    shoot_pub_ = this->create_publisher<helios_control_interfaces::msg::ShooterCmd>("/shooter_cmd", 10);
    imu_sub_ = this->create_subscription<sensor_interfaces::msg::ImuEuler>(
            "/imu_euler_out", 10, std::bind(&FireController::imu_euler_callback, this, std::placeholders::_1));
    // bullet_sub_ = this->create_subscription<referee_interfaces::msg::>(, , )
    target_sub_ = this->create_subscription<autoaim_interfaces::msg::Target>(
            "/predictor/target", 10, std::bind(&FireController::target_callback, this, std::placeholders::_1));
    // create process thread
    timer_ = this->create_wall_timer(5ms, std::bind(&FireController::target_process, this));
}

FireController::~FireController() {

}

void FireController::imu_euler_callback(sensor_interfaces::msg::ImuEuler::SharedPtr imu_msg) {
    // update
    imu_ypr_ = Eigen::Vector3d{imu_msg->yaw, imu_msg->pitch, imu_msg->roll};
}

void FireController::target_callback(autoaim_interfaces::msg::Target::SharedPtr target_msg) { 
    target_msg_ = target_msg;
}

// void FireController::serial_callback(autoaim_interfaces::msg::ReceiveData::SharedPtr serial_msg) {
//     imu_ypr_(0) = serial_msg->yaw;
//     imu_ypr_(1) = serial_msg->pitch;
//     imu_ypr_(2) = 0;
//     bullet_solver_->update_bullet_speed(serial_msg->bullet_speed);
// }

void FireController::target_process() {
    if (!target_msg_) {
        // send data when under traditional mode
        gimbal_cmd_.header.stamp = this->now();
        gimbal_cmd_.yaw_value = 10;
        gimbal_cmd_.pitch_value = 0;
        gimbal_cmd_.gimbal_mode = 1;
        gimbal_pub_->publish(gimbal_cmd_);
        return;
    } else {
        if (!target_msg_->tracking) {
            // send data when under traditional mode
            gimbal_cmd_.header.stamp = this->now();
            gimbal_cmd_.yaw_value = 10;
            gimbal_cmd_.pitch_value = 0;
            gimbal_cmd_.gimbal_mode = 1;
            gimbal_pub_->publish(gimbal_cmd_);
            return;
        }
    }
    // check if msg has expired
    if ((this->now() - target_msg_->header.stamp).seconds() > params_.message_expire_time) {
        return;
    }
    // caculate new position
    Eigen::Vector3d predicted_xyz = target_solver_->get_best_armor(target_msg_, imu_ypr_(0), latency_);
    // caculate fly time and pitch compensation
    double fly_time = 0;
    // caculate compute time cost
    double total_latency = (this->now() - target_msg_->header.stamp).seconds();
    if (params_.debug) {
        RCLCPP_INFO(logger_, "total latency: %f", total_latency);
    }
    // update gimbal cmd
    gimbal_cmd_.header.stamp = this->now();
    gimbal_cmd_.yaw_value = -std::atan2(predicted_xyz(1), predicted_xyz(0)) * 180 / M_PI;
    gimbal_cmd_.pitch_value = bullet_solver_->iterate_pitch(predicted_xyz, fly_time) - 5.9;
    gimbal_cmd_.gimbal_mode = 0;
    gimbal_pub_->publish(gimbal_cmd_);
    // update shoot cmd
    shooter_cmd_.header.stamp = this->now();
    shooter_cmd_.shooter_speed = 2;
    // judge shoot cmds
    bool shoot_cmd = judge_shoot_cmd(predicted_xyz.norm(), target_solver_->best_armor_yaw_); 
    shooter_cmd_.fire_flag = shoot_cmd ? 1 : 0;
    // shooter_cmd_.fire_flag = 0;
    shooter_cmd_.dial_vel = 1;
    // RCLCPP_WARN(logger_, "yaw %f, pitch %f", gimbal_cmd_.yaw_value, gimbal_cmd_.pitch_value);
    shoot_pub_->publish(shooter_cmd_);
    // update predict latency
    latency_ = params_.latency + fly_time + total_latency;
}

bool FireController::judge_shoot_cmd(double distance, double armor_yaw) {
    double armor_width = 0.135f;
    double armor_height = 0.125f;
    double yaw_error_threshold = std::fabs(std::atan2(armor_width / 2.0f, distance));
    double pitch_error_threshold = std::fabs(std::atan2(armor_height / 2.0f, distance)) * 3;
    RCLCPP_WARN(logger_, "distance %f", distance);
    Eigen::Vector3d car_center_ypd = target_solver_->get_car_center_ypd(target_msg_);
    RCLCPP_WARN(logger_, "diff %f, thresh %f", std::fabs(angles::shortest_angular_distance(armor_yaw, car_center_ypd[0])), M_PI / 3);
    RCLCPP_WARN(logger_, "yaw diff %f thresh %f",angles::shortest_angular_distance(angles::from_degrees(gimbal_cmd_.yaw_value), angles::from_degrees(imu_ypr_[0])), yaw_error_threshold);
    RCLCPP_WARN(logger_, "pitch diff %f thresh %f",std::fabs(angles::shortest_angular_distance(angles::from_degrees(gimbal_cmd_.pitch_value), angles::from_degrees(imu_ypr_[1]))), pitch_error_threshold);

    if (std::fabs(angles::shortest_angular_distance(angles::from_degrees(gimbal_cmd_.yaw_value), angles::from_degrees(imu_ypr_[0])) < yaw_error_threshold &&
        std::fabs(angles::shortest_angular_distance(angles::from_degrees(gimbal_cmd_.pitch_value), angles::from_degrees(imu_ypr_[1]))) < pitch_error_threshold &&
        std::fabs(angles::shortest_angular_distance(armor_yaw, car_center_ypd[0])) < M_PI / 4)) {
        return true;
    } else {
        return false;    
    }
}

} // namespace helios_cv


// register node to component
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::FireController);