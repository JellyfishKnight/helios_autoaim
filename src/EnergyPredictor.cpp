// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "EnergyPredictor.hpp"
#include "CeresSolver.hpp"
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

}

void EnergyPredictor::set_params(helios_autoaim::Params::Predictor predictor_params) {
    predictor_params_ = predictor_params.energy_predictor;
}

std::vector<double> EnergyPredictor::get_state() const {
    
}

void EnergyPredictor::EnergyRefresh(){
    /** clear vector container **/
    angle.clear();
    filter_omega.clear();
    time_series.clear();
    t_list.clear();
    total_theta = 0;
    /** init state **/
    circle_mode = INIT;
    omega_kf_flag = false;
    clockwise_cnt = 0;
    fit_cnt = 0;    // curve fitting times
    predict_pts.resize(4);
    average_v_bullet = 27;
    v_vec[0] = average_v_bullet;
    isSolve=false;
    refresh=true;
    time_fin=0;
    time_start=0;
    time_pre=0;
    phi_ = CV_2PI/2;
    w_ = 1.942;//1.884~2.0
    a_ = 0.9125;//0.78~1.045
    st_ = 15;
    initFanRotateKalman();
}

inline int EnergyPredictor::round2int(float f) {
        int int_f = f;
        float d = f - int_f;
        if(d >= 0.5) return int_f+1;
        else if(d>0 && d < 0.5) return int_f;
        else if(d<0 && d > -0.5) return int_f;
        else return int_f - 1;
}

/**
 * @brief 求出角度增加值
*/
float EnergyPredictor::calOmegaNstep(int step, float &total_theta){
    long unsigned int step_ = step+1;
    if(angle.size()<step_){
        return 0;
    }else{
        float d_theta;
        float dt = t_list.back()-t_list[t_list.size()-step_];

        float last_d_theta = angle[angle.size()-1] - angle[angle.size()-2];

        if(last_d_theta>6.1){
            last_d_theta -= CV_2PI;
        }else if(last_d_theta<-6.1){
            last_d_theta += CV_2PI;
        }    
        int d_fan = round2int(last_d_theta / 1.2566);//1.2566是pi的一半

        if(abs(d_fan)>=1){
            for(long unsigned int i=2; i<=step_; i++){
                angle[angle.size()-i]+=d_fan * 1.2566;
                if(angle[angle.size()-i] < -CV_2PI) angle[angle.size()-i] += CV_2PI;
                if(angle[angle.size()-i] > CV_2PI) angle[angle.size()-i] += -CV_2PI;
            }
        }

        d_theta = angle[angle.size()-1]-angle[angle.size()-step_];

        if(d_theta > 6){
            d_theta -= CV_2PI;
        }
        if(d_theta < -6){
            d_theta += CV_2PI;
        }
        total_theta += d_theta;

        float tmp_omega = d_theta / dt;

        if(fabs(tmp_omega)>2.1){
            if(omega_kf_flag){//这次用卡尔曼插值
                total_theta = omega_kf.state_post_[0];
                tmp_omega = omega_kf.state_post_[1] + omega_kf.state_post_[2]*(dt/step);
            }
            else{
                tmp_omega = (tmp_omega>0) ? 2.09 : -2.09;
            }
        }
        return tmp_omega;
    }
}

/*
    判断顺逆时针
*/
inline void EnergyPredictor::JudgeFanRotation(){
    clockwise_cnt = (current_omega>0)? clockwise_cnt+1 : clockwise_cnt-1;
    energy_rotation_direction = (clockwise_cnt>0)? 1 : -1;
    //cout<<"rotation: "<<energy_rotation_direction<<endl;
}

void EnergyPredictor::FilterOmega(const float &dt){
    omega_kf.trans_mat_ << 1, dt, 0.5 * dt * dt,
                           0,  1, dt,
                           0,  0,  1;
    Eigen::VectorXf measure_vec(2, 1);
    measure_vec << total_theta, current_omega;
    omega_kf.predict();
    omega_kf.correct(measure_vec);
    omega_kf_flag = true;
    filter_omega.push_back(energy_rotation_direction*omega_kf.state_post_[1]);
}

bool EnergyPredictor::EnergyStateSwitch(){
    switch(circle_mode){
        case INIT:
            if(FindWavePeak()){//找到第一个波峰来确定一个phi值，然后初始化并进入下一阶段
                circle_mode = STANDBY;
                fit_cnt = 0;
                change_cnt = 0;
                max_omega = 0;
                min_omega = 5;
                isSolve=false;
            }
            return false;
        case STANDBY:
            if(time_series.back() - time_series[st_] > 1.5){
                circle_mode = ESTIMATE;
            }
            return false;
        case ESTIMATE:
            ceres_cnt++;
            isSolve=true;
            //estimateParam(filter_omega,time_series);//最小二乘拟合
            //a=0;
            circle_mode = PREDICT;
            return true;
        case PREDICT:
            isSolve=false;
            return true;
        default:
            return true;

    }
}

/**
 * @brief 找到第一个波峰来确定一个初始的phi
*/
bool EnergyPredictor::FindWavePeak(){
    if (filter_omega.size() > 15) {
        std::vector<float> cut_filter_omega(filter_omega.end() - 7, filter_omega.end());
        std::vector<float> cut_time_series(time_series.end() - 7, time_series.end());
        Eigen::MatrixXd rate = LeastSquare(cut_time_series, cut_filter_omega, 1);
        fit_cnt = (rate(0, 0) > 0) ? fit_cnt + 1 : fit_cnt - 1;
        if (fit_cnt > 5) {
            if (fabs(filter_omega.back()) > max_omega) {
                change_cnt = 0;
                max_omega = filter_omega.back();
                phi_ = CV_2PI / 2 - w_ * time_series.back();
                while (phi_ < -CV_PI || phi_ > CV_PI) {
                    phi_ += 2 * CV_PI;
                }
            } else {
                change_cnt++;
            }
        } else if (fit_cnt < -5) {
            if (fabs(filter_omega.back()) < min_omega) {
                change_cnt = 0;
                min_omega = filter_omega.back();
                phi_ = -CV_PI / 2 - w_ *time_series.back();
                while (phi_ < -CV_PI || phi_ > CV_PI) {
                    phi_ += 2 * CV_PI;
                }
            } else {
                change_cnt++;
            }
        } else {
            return false;
        }
        if (change_cnt > 5) {
            // if (fit_cnt >= 0) {
            //     // cout<<"--- get peak omega :"<<max_omega<<"   init phi :"<<phi_<<endl;
            // } else {
            //     // cout<<"--- get valley omega :"<<min_omega<<"   init phi :"<<phi_<<endl;
            // }
            return true;
        } else {
            return false;
        }
    } else {
        return false; 
    }
}


/*
    最小二乘法
*/
Eigen::MatrixXd EnergyPredictor::LeastSquare(std::vector<float> x, std::vector<float>y, int N){
    Eigen::MatrixXd A(x.size(), N + 1);
    Eigen::MatrixXd B(y.size(), 1);
    Eigen::MatrixXd W;

    for (unsigned int i = 0; i < x.size(); ++i) {
        for (int n = N, dex = 0; n >= 1; --n, ++dex) {
            A(i, dex) = pow(x[i], 2);
        }
        A(i, N) = 1;
        B(i, 0) = y[i];
        W = (A.transpose() * A).inverse() * A.transpose() * B;

        return W;
    }
}

inline float EnergyPredictor::IdealOmega(float &t_){
    return a_ *sin(w_*t_+phi_) + 2.09 - a_;//2.09
}


float EnergyPredictor::IdealRad(float t1, float t2){
    return (-a_/w_) * (cos(w_*t2 + phi_) - cos(w_*t1 + phi_)) + (2.09-a_)*(t2-t1);
}


cv::Point2f EnergyPredictor::calPredict(cv::Point2f &p, cv::Point2f &center, float theta) const {
    Eigen::Matrix2f rotate_matrix;
    Eigen::Vector2f cur_vec, pre_vec;
    float c = cos(theta), s = sin(theta);
    rotate_matrix << c, -s,
                     s,  c; //旋转矩阵
    cur_vec << (p.x-center.x), (p.y-center.y);
    pre_vec = rotate_matrix * cur_vec;
    return cv::Point2f(center.x + pre_vec[0], center.y + pre_vec[1]);
}

void EnergyPredictor::getPredictRect(cv::Point2f &center, std::vector<cv::Point2f>& pts, float theta) {
    for (int i = 0; i < 4; i++) {
        predict_pts[i] = calPredict(pts[i], center, theta);
    }
}


void EnergyPredictor::initFanRotateKalman() {
    omega_kf.Init(3, 2, 1);
    omega_kf.measure_mat_.setIdentity();
    omega_kf.process_noise_.setIdentity();
    
    omega_kf.process_noise_<<1, 0, 0,
                            0, 1, 0,
                            0, 0, 1;
    // 测量噪声协方差矩阵R
    omega_kf.measure_noise_.setIdentity();
    omega_kf.measure_noise_ <<  20, 0,      // TODO improve rotate kalman noise parameters
                                 0, 20;
    // 误差估计协方差矩阵P
    omega_kf.error_post_.setIdentity();
    omega_kf.state_post_ << current_theta,
            current_omega,
            0;
}

void EnergyPredictor::Set_problem_parameter(){
    Omega problem;

    problem.a = a_;
    problem.phi = phi_;
    problem.w=w_;
    problem.t = t_list[t_list.size()-1-differ_step/2];
    problem.st = st_;
    problem.omega  =energy_rotation_direction*omega_kf.state_post_[1];
    problem.solve = isSolve;
    problem.refresh=refresh;

    // ceres_problem->publish(problem);
    refresh=false;
    problem.refresh=false;
}


} // namespace helios_cv