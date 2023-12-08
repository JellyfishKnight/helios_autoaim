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
#include <angles/angles.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "autoaim_interfaces/msg/target.hpp"

namespace helios_cv {

#define BALANCE_ARMOR_NUM 2
#define SIMPLE_ARMOR_NUM 4
#define OUTPOST_ARMOR_NUM 3

class TargetSolver {
public:
    TargetSolver() = default;

    ~TargetSolver() = default;

    Eigen::Vector3d get_best_armor(autoaim_interfaces::msg::Target::SharedPtr target_msg, double now_yaw, double latency);

    Eigen::Vector3d get_car_center_ypd(autoaim_interfaces::msg::Target::SharedPtr target_msg);

    double best_armor_yaw_;
private:
    std::vector<Eigen::Vector3d> armors_position_;
    std::vector<double> armors_yaw_;

    autoaim_interfaces::msg::Target new_target_;
    Eigen::Vector3d car_center_xyz_;
    Eigen::Vector3d car_velocity_xyz_;
    double dz_;
    double yaw_;
    double v_yaw_;
    double r1_;
    double r2_;
    int a_n_;
    bool is_current_pair_;

    int best_armor_idx_;

    double radius_;
};

} // helios_cv