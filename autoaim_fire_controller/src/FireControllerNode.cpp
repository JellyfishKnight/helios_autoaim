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
    bullet_solver_->update_bullet_params(BulletParams{
        params_.bullet_solver.bullet_speed,
        params_.bullet_solver.bullet_mass,
        params_.bullet_solver.air_coeff
    });
    // create publisher and subscription
    gimbal_pub_ = this->create_publisher<helios_control_interfaces::msg::GimbalCmd>("gimbal_cmd_", 10);
    shoot_pub_ = this->create_publisher<helios_control_interfaces::msg::ShooterCmd>("shooter_cmd", 10);
    imu_sub_ = this->create_subscription<sensor_interfaces::msg::ImuEuler>(
            "imu_euler", 10, std::bind(&FireController::imu_euler_callback, this, std::placeholders::_1));
    // bullet_sub_ = this->create_subscription<referee_interfaces::msg::>(, , )
    // // 初始化tf2相关
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // // Create the timer interface before call to waitForTransform,
    // // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // // subscriber and filter
    armors_sub_.subscribe(this, "/predictor/target", rmw_qos_profile_sensor_data);
    tf2_filter_ = std::make_shared<tf2_filter>(
        armors_sub_, *tf2_buffer_, params_.target_frame, 10, this->get_node_logging_interface(),
        this->get_node_clock_interface(), std::chrono::duration<int>(2));
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&FireController::target_callback, this);        
    // create process thread
    std::thread{
        [this]() {
            while (rclcpp::ok()) {
                target_process();
            }
        }
    }.detach();
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

void FireController::serial_callback(autoaim_interfaces::msg::ReceiveData::SharedPtr serial_msg) {
    imu_ypr_(0) = serial_msg->yaw;
    imu_ypr_(1) = serial_msg->pitch;
    imu_ypr_(2) = 0;
    bullet_solver_->update_bullet_speed(serial_msg->bullet_speed);
}

void FireController::target_process() {
    if (!target_msg_->tracking || !target_msg_) {
        // send data when under traditional mode
        return;
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
    gimbal_cmd_.yaw = std::atan2(predicted_xyz(1), predicted_xyz(0));
    gimbal_cmd_.pitch = bullet_solver_->iterate_pitch(predicted_xyz, fly_time);
    gimbal_cmd_.gimbal_mode = 1;
    gimbal_pub_->publish(gimbal_cmd_);
    // update shoot cmd
    shooter_cmd_.header.stamp = this->now();
    shooter_cmd_.shooter_speed = 2;
    // judge shoot cmds
    bool shoot_cmd = judge_shoot_cmd(predicted_xyz.norm(), target_solver_->best_armor_yaw_); 
    shooter_cmd_.fire_flag = shoot_cmd ? 1 : 0;
    shooter_cmd_.dial_vel = 10;
    shoot_pub_->publish(shooter_cmd_);
    // update predict latency
    latency_ = params_.latency + fly_time + total_latency;
}

bool FireController::judge_shoot_cmd(double distance, double armor_yaw) {
    double armor_width = 0.135f;
    double armor_height = 0.125f;
    double yaw_error_threshold = std::fabs(std::atan2(armor_width / 2.0f, distance));
    double pitch_error_threshold = std::fabs(std::atan2(armor_height / 2.0f, distance));
    Eigen::Vector3d car_center_ypd = target_solver_->get_car_center_ypd(target_msg_);
    if (std::fabs(angles::shortest_angular_distance(gimbal_cmd_.yaw, angles::from_degrees(imu_ypr_[0])) < yaw_error_threshold &&
        std::fabs(angles::shortest_angular_distance(gimbal_cmd_.pitch, angles::from_degrees(imu_ypr_[1]))) < pitch_error_threshold &&
        std::fabs(angles::shortest_angular_distance(armor_yaw, car_center_ypd[0])) < M_PI / 5)) {
        return true;
    } else {
        return false;    
    }
}

} // namespace helios_cv