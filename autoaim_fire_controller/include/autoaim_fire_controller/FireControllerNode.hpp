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
#pragma once

#include <rclcpp/rclcpp.hpp>


#include "autoaim_interfaces/msg/target.hpp"
#include "autoaim_interfaces/msg/receive_data.hpp"
#include "helios_control_interfaces/msg/shooter_cmd.hpp"
#include "helios_control_interfaces/msg/gimbal_cmd.hpp"
#include "sensor_interfaces/msg/imu_euler.hpp"

#include "TargetSolver.hpp"
#include "BulletSolver.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <message_filters/subscriber.h>

// auto generated by ros2 generate_parameter_library
// https://github.com/PickNikRobotics/generate_parameter_library
#include "fire_controller_parameters.hpp"

namespace helios_cv {

using tf2_filter = tf2_ros::MessageFilter<autoaim_interfaces::msg::Target>;
using ParamListener = fire_controller::ParamListener;
using Params = fire_controller::Params;

class FireController : public rclcpp::Node {
public:
    FireController(const rclcpp::NodeOptions& options);

    ~FireController();
private:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;
    // tf2 
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<autoaim_interfaces::msg::Target> armors_sub_;
    std::shared_ptr<tf2_filter> tf2_filter_;
    std::shared_ptr<BulletSolver> bullet_solver_;
    std::shared_ptr<TargetSolver> target_solver_;

    rclcpp::Subscription<autoaim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_interfaces::msg::ImuEuler>::SharedPtr imu_sub_;
    // rclcpp::Subscription<referee_interfaces::msg::BulletSpeed>::SharedPtr bullet_speed_sub_;
    rclcpp::Publisher<helios_control_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
    rclcpp::Publisher<helios_control_interfaces::msg::ShooterCmd>::SharedPtr shoot_pub_;

    void target_callback(autoaim_interfaces::msg::Target::SharedPtr target_msg);

    void imu_euler_callback(sensor_interfaces::msg::ImuEuler::SharedPtr imu_msg);

    void serial_callback(autoaim_interfaces::msg::ReceiveData::SharedPtr serial_msg);

    double latency_ = 0.15;

    // void bullet_speed_callback();
    
    void target_process();

    bool judge_shoot_cmd(double distance, double yaw, double pitch);

    Eigen::Vector3d xyz2ypd(const Eigen::Vector3d &_xyz);

    autoaim_interfaces::msg::Target::SharedPtr target_msg_;
    Eigen::Vector3d ypr_ = Eigen::Vector3d::Zero();

    rclcpp::Logger logger_ = rclcpp::get_logger("fire_controller");
};


} // namespace helios_cv