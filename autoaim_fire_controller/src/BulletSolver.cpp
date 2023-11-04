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
#include "BulletSolver.hpp"

namespace helios_cv {

double BulletSolver::iterate_pitch(Eigen::Vector3d target_xyz, double& fly_time) {
    if(bullet_speed_ < 11 || bullet_speed_ > 35) {
        bullet_speed_ = 20;
    }
    float x = target_xyz[0] ,y = target_xyz[1], z = target_xyz[2];
    float d = sqrt(x*x + y*y);
    float h = z;
    float d_ = d, h_ = h, dh = 1; // temp value
    float pitch;
    float v_y0, v_z0;
    int i = 0;
    //误差不满足要求时迭代计算
    while(abs(dh) > 1e-02){
        i++;
        pitch = atan2(h_,d_);
        v_y0 = bullet_speed_ * cos(pitch);
        v_z0 = bullet_speed_ * sin(pitch);
        fly_time = (exp(params_.air_coeff * d) - 1) / v_y0 / params_.air_coeff;
        float temp_h = v_z0 * fly_time - 0.5 * gravity_ * fly_time * fly_time;
        dh = h - temp_h;
        h_ += dh;
        if(i > 10) {
            RCLCPP_WARN(rclcpp::get_logger("fire_controller"), "Error still exists");
            break;
        }
    }
    pitch /= (M_PI / 180.0);
    // std::cout << "interate pitch: " << pitch << std::endl;
    return pitch;
}

void BulletSolver::update_bullet_speed(double received_speed) {
    if (!check_bullet_velocity(received_speed) && v_vec_[(v_vec_pointer_ + 3) % 4] != received_speed) {
        v_vec_[v_vec_pointer_++ % 4] = received_speed;
        bullet_speed_ = 0;
        for (int i = 0; i < 4; i++) {
            bullet_speed_ += v_vec_[i];
        }
        bullet_speed_ /= 4.0;
    } else {
        bullet_speed_ = 30.0;
    }
}

bool BulletSolver::check_bullet_velocity(double bullet_speed) {
    if (bullet_speed < 25 || bullet_speed > 35) {
        return false;
    }
    return true;
}

void BulletSolver::update_bullet_params(const BulletParams& bullet_params) {
    params_ = bullet_params;
    bullet_speed_ = check_bullet_velocity(bullet_params.bullet_speed) ? 
                                                        bullet_params.bullet_speed : 30.0;
}

} // namespace helios_cv