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

#include <Eigen/Core>
#include <Eigen/Dense>

#include "autoaim_interfaces/msg/target.hpp"

namespace helios_cv {

class TargetSolver {
public:
    TargetSolver() = default;

    ~TargetSolver() = default;

    void caculate_new_state(autoaim_interfaces::msg::Target::SharedPtr target_msg);

    void get_best_armor(Eigen::Vector3d car_center, double yaw, double r, Eigen::Vector3d& best_armor);

};

} // helios_cv