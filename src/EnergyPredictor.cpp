// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "EnergyPredictor.hpp"
#include "LeastSquares.hpp"
#include <rclcpp/time.hpp>

namespace helios_cv {
EnergyPredictor::EnergyPredictor(helios_autoaim::Params::Predictor::EnergyPredictor predictor_params) {
    predictor_params_ = predictor_params;
}

void EnergyPredictor::set_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr cam_info) {
    cam_info_ = cam_info;
    cam_center_ = cv::Point2f(cam_info_->k[2], cam_info_->k[5]);
    pnp_solver_ = std::make_shared<PnPSolver>(cam_info->k, cam_info->d);
}

void EnergyPredictor::init_predictor(helios_autoaim::Params::Predictor predictor_param, tf2_ros::Buffer::SharedPtr tf_buffer) {
    predictor_params_ = predictor_param.energy_predictor;
    tf_buffer_ = tf_buffer;
}

helios_rs_interfaces::msg::Target EnergyPredictor::predict_target(helios_rs_interfaces::msg::Armors armors, const rclcpp::Time& now) {
    energy_pts_.emplace_back(cv::Point2f(armors.points[0].x, armors.points[0].y))
    energy_pts_.emplace_back(cv::Point2f(armors.points[1].x, armors.points[1].y))
    energy_pts_.emplace_back(cv::Point2f(armors.points[2].x, armors.points[2].y))
    energy_pts_.emplace_back(cv::Point2f(armors.points[3].x, armors.points[3].y))
    energy_pts_.emplace_back(cv::Point2f(armors.points[4].x, armors.points[4].y))
    omega_.set_time(now);

}

void EnergyPredictor::set_params(helios_autoaim::Params::Predictor predictor_params) {
    predictor_params_ = predictor_params.energy_predictor;
}

std::vector<double> EnergyPredictor::get_state() const {
    
}

void EnergyPredictor::refresh(){
    omega_.refresh();

    circle_mode_ = INIT;

    predict_point_.resize(4);
    energy_pts_.resize(4);
    isSolve_ = false;
    reFresh_ = true;
    latency_ = 0.6;
    fly_t_ = 0.2;
    react_t_ = 0.15;
    ceres_cnt_ = 1;

    refresh_compensation();
    omega_kf_.initKalman();
}

void EnergyPredictor::energy_predict(uint8_t mode, std::vector<cv::Point2f> &energy_points, cv::Point2f &center){
    cv::Point2f target_point;
    if(energy_points.size() == 4){
        target_point = cv::Point2f((energy_points[0].x + energy_points[2].x) / 2, (energy_points[0].y + energy_points[2].y) / 2);
    }else{
        return ;
    }

    omega_.set_theta(std::atan2((target_point - center).y, (target_point - center).x));
    if(omega_.start == false){
        predict_rad_ = 0;
    }else{
        if(mode == BIG_ENERGY){
            FilterOmega();

            
        }
    }
}

void EnergyPredictor::FilterOmega(float &dt){
    omega_.trans_mat_ <<1, dt, 0.5 * dt * dt,
                        0, 1, dt,
                        0, 0, 1;
    Eigen::VectorXf measure_vec(2, 1);
    measure_vec << omega_.total_theta_, omega_.current_theta_;
    omega_kf_.predict();
    omega_kf_.correct(measure_vec);
    omega_.set_filter(omega_kf_.state_post_[1]);

}

bool EnergyPredictor::EnergyStateSwitch(){
    switch(circle_mode){
        case INIT:
            if(omega_.FindWavePeak()){
                circle_mode = STANDBY;
                isSolve = false;
                omega_.refresh_after_wave();
            }
            return false;
        case STANDBY:
            if(omega_.get_time_gap() > 1.5){
                circle_mode = ESTIMATE;
            }
            return false;
        case ESTIMATE:
            ceres_cnt_++;
            isSolve = true;
            circle_mode = PREDICT;
            return true;
        case PREDICT:
            isSolve = false;
            return true;
        default:
            return true;
    }
}

void EnergyPredictor::calPredict(cv::Point2f &p, cv::Point2f &center, float theta)const{
    Eigen::Matrix2f rotate_matrix;
    Eigen::Vector2f cur_vec, pre_vec;

    float c = std::cos(theta), s = std::sin(theta);
    rotate_matrix<< c, -s,
                    s, c;
    cur_vec << (p.x - center.x), (p.y - center.y);
    pre_vec = rotate_matrix * cur_vec;

    return cv::Point2f(center.x + pre_vec[0], center.y + pre_vec[1]);
}

void EnergyPredictor::getPredictRect(cv::Point2f &center, std::vector<cv::Point2f> &pts, float theta){
    for(int i = 0; i < 4; i++){
        predict_point[i] = calPredict(pts[i], center, theta);
    }
}


} // namespace helios_cv