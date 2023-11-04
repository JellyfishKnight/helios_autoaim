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

#include <rclcpp/rclcpp.hpp>
namespace helios_cv {


typedef struct BulletParams{
    double bullet_speed;
    double bullet_mass;
    double air_coeff;
}BulletParams;


class BulletSolver {
public:
    BulletSolver() = default;

    ~BulletSolver() = default;

    double iterate_pitch(Eigen::Vector3d target_xyz, double& fly_time);

    void update_bullet_speed(double received_speed);

    void update_bullet_params(const BulletParams& bullet_params);

    bool check_bullet_velocity(double bullet_speed);
private:
    BulletParams params_;

    double v_vec_[4];
    double bullet_speed_;
    int v_vec_pointer_ = 1;
    double gravity_ = 9.8;
};


} // namespace helios_cv