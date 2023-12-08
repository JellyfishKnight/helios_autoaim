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
#include "TargetSolver.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace helios_cv {

Eigen::Vector3d TargetSolver::get_best_armor(autoaim_interfaces::msg::Target::SharedPtr target_msg, double now_yaw, double latency) {
    // resize vector
    armors_position_.resize(target_msg->armors_num, Eigen::Vector3d::Zero());
    armors_yaw_.resize(target_msg->armors_num, 0);
    // 
    car_center_xyz_ = Eigen::Vector3d(target_msg->position.x, target_msg->position.y, target_msg->position.z);
    car_velocity_xyz_ = Eigen::Vector3d(target_msg->velocity.x, target_msg->velocity.y, target_msg->velocity.z);
    dz_ = target_msg->dz;
    yaw_ = target_msg->yaw;
    v_yaw_ = target_msg->v_yaw;
    r1_ = target_msg->radius_1;
    r2_ = target_msg->radius_2;
    a_n_ = target_msg->armors_num;
    is_current_pair_ = true;
    // caculate predicted params of car
    car_center_xyz_ += car_velocity_xyz_ * latency;
    yaw_ += v_yaw_ * latency;
    // caculate armors position
    if (a_n_ == BALANCE_ARMOR_NUM) {
        for (int i = 0; i < 2; i++) {
            float tmp_yaw = yaw_ + i * M_PI;
            float r = r1_;
            armors_position_[i](0) = car_center_xyz_(0) - r * cos(tmp_yaw);
            armors_position_[i](1) = car_center_xyz_(1) - r * sin(tmp_yaw);
            armors_position_[i](2) = car_center_xyz_(2);
            armors_yaw_[i] = tmp_yaw;
        }
    } else if (a_n_ == OUTPOST_ARMOR_NUM) {
        for (int i = 0; i < 3; i++) {
            float tmp_yaw = yaw_ + i * 2.0 * M_PI / 3.0;
            // The radius of outpost is fixed
            float r =  0.26;
            armors_position_[i](0) = car_center_xyz_(0) - r * cos(tmp_yaw);
            armors_position_[i](1) = car_center_xyz_(1) - r * sin(tmp_yaw);
            armors_position_[i](2) = car_center_xyz_(2);
            armors_yaw_[i] = tmp_yaw;
        }
    } else if (a_n_ == SIMPLE_ARMOR_NUM) {
        for (int i = 0; i < 4; i++) {
            float tmp_yaw = yaw_ + i * M_PI / 2.0;
            float r = is_current_pair_ ? r1_ : r2_;
            armors_position_[i](0) = car_center_xyz_(0) - r * cos(tmp_yaw);
            armors_position_[i](1) = car_center_xyz_(1) - r * sin(tmp_yaw);
            armors_position_[i](2) = is_current_pair_ ? car_center_xyz_(2) : car_center_xyz_(2) + dz_;
            armors_yaw_[i] = tmp_yaw;
            is_current_pair_ = !is_current_pair_;
        }
    }
    Eigen::Vector3d car_center_ypd = get_car_center_ypd(target_msg);
    best_armor_idx_ = 0;
    double yaw_diff_min = std::fabs(angles::shortest_angular_distance(car_center_ypd[0], armors_yaw_[0]));
    for (int i = 1; i < 4; i++) {
        double temp_yaw_diff = std::fabs(angles::shortest_angular_distance(car_center_ypd[0], armors_yaw_[i]));
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            best_armor_idx_ = i;
        }
    }
    best_armor_yaw_ = armors_yaw_[best_armor_idx_];
    return armors_position_[best_armor_idx_];
}

Eigen::Vector3d get_car_center_ypd(autoaim_interfaces::msg::Target::SharedPtr target_msg) {
    float x = target_msg->position.x, y = target_msg->position.y, z = target_msg->position.z;
    double car_center_yaw = -std::atan2(-y, x);
    double car_center_pitch = std::atan2(z, std::sqrt(y * y + x * x)); 
    double car_center_distance = std::sqrt(x * x + y * y + z * z);
    return Eigen::Vector3d(car_center_yaw, car_center_pitch, car_center_distance);
}

} // helios_cv