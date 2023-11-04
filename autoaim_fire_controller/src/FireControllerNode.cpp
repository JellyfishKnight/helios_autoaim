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


namespace helios_cv {

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
    // create publisher
    serial_pub_ = this->create_publisher<autoaim_interfaces::msg::SendData>("serial_send", 10);
    gimbal_pub_ = this->create_publisher<helios_control_interfaces::msg::GimbalCmd>("gimbal_cmd", 10);
    shoot_pub_ = this->create_publisher<helios_control_interfaces::msg::ShooterCmd>("shooter_cmd", 10);
    // create subscriber
    target_sub_ = this->create_subscription<autoaim_interfaces::msg::Target>(
        "target", 10, std::bind(&FireController::target_callback, this, std::placeholders::_1));
    // create process thread
    std::thread{
        [this]() {
            rclcpp::Rate rate(300);
            while (rclcpp::ok()) {
                target_process();
                rate.sleep();
            }
        }
    }.detach();
}

FireController::~FireController() {

}

void FireController::target_callback(const autoaim_interfaces::msg::Target::SharedPtr target_msg) { 
    target_msg_ = target_msg;
}

void FireController::target_process() {
    if (!target_msg_) {
        RCLCPP_WARN(logger_, "null target msg");
        return ;
    }
    // caculate new position
    Eigen::Vector3d target_xyz, target_vel, new_xyz;
    target_solver_->caculate_new_state();


    // update gimbal cmd
    auto gimbal_cmd = std::make_unique<helios_control_interfaces::msg::GimbalCmd>();
    // gimbal_cmd->yaw = target_solver_->get_yaw();
    // gimbal_cmd->pitch = target_solver_->get_pitch();
    gimbal_pub_->publish(std::move(gimbal_cmd));
    // update shoot cmd
    auto shoot_cmd = std::make_unique<helios_control_interfaces::msg::ShooterCmd>();
    // shoot_cmd->shoot_cmd = target_msg_.shoot_cmd;
    shoot_pub_->publish(std::move(shoot_cmd));
    // update serial send
    auto serial_send = std::make_unique<autoaim_interfaces::msg::SendData>();
    // serial_send->send_data = target_msg_.send_data;
    serial_pub_->publish(std::move(serial_send));
}


} // namespace helios_cv