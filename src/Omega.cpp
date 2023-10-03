#include"Omega.hpp"


/**
 * @brief 初始化
*/
Omega::Omega(){
    refresh();
}

/**
 * @brief 找到波峰后的初始化
*/
void Omega::refresh_after_wave(){
    fit_cnt_ = 0;
    change_cnt_ = 0;
    max_omega_ = 0;
    min_omega_ = 5;
}

/**
 * @brief 刷新数据
*/
void Omega::refresh(){
    angle_.clear();
    filter_omega_.clear();
    time_series_.clear();
    t_list_.clear();
    clockwise_cnt_ = 0;
    energy_rotation_direction_ = 1;
    time_start_ = 0;
    time_gap_ = 0;
    a_ = 0.9125;
    w_ = 1.942;
    phi_ = CV_2PI/2;
    st_ = 15;
    start_ = false;
    dt_ = 0.012;
    differ_step_ = 4;
    refresh_after_wave();

}


/**
 * @brief 设置角度， 满足条件的开始求解角速度
 * @param theta 两帧之间的角度
*/
void Omega::set_theta(float theta){
    angle_.push_back(theta);
    if(angle_.size() > differ_step_){
        start_ = true;
        current_theta_ = calOmegaNstep(differ_step_, total_theta_);
        JudgeFanRotation(current_theta_);
    }

    change_time_series();
}


/**
 * @brief 根据传入的角度求解角速度
 * @param step 指定步长多少来平滑
 * @param total_theta 角度
*/
float Omega::calOmegaNstep(int step, float &total_theta){
    long unsigned int step_ = step + 1;
    if(angle_.size() < step_){
        return 0;
    }

    float d_theta;
    float dt = t_list_.back() - t_list_[t_list_.size() - step_];

    float last_d_theta = angle_[angle_.size() - 1] - angle_[angle_.size() - 2];

    if(last_d_theta > 6.1){
        last_d_theta -= CV_2PI;
    }else if(last_d_theta < -6.1){
        last_d_theta += CV_2PI;
    }

    int d_fan = round2int(last_d_theta / 1.2566);

    if(abs(d_fan) >= 1){
        for(long unsigned int i = 2; i <= step_; i++){
            angle_[angle_.size() - i] += d_fan * 1.2566;
            if(angle_[angle_.size() - i] < -CV_2PI) angle_[angle_.size() - i] += CV_2PI;
            if(angle_[angle_.size() - i] > CV_2PI) angle_[angle_.size() - i] += -CV_2PI;
        }
    }

    d_theta = angle_[angle_.size() - 1] - angle_[angle_.size() - step_];

    if(d_theta > 6){
        d_theta -= CV_2PI;
    }
    if(d_theta < -6){
        d_theta += CV_2PI;
    }

    float tmp_omega = d_theta / dt;

    if(fabs(tmp_omega) > 2.1){
        tmp_omega = (tmp_omega > 0) ? 2.09 : -2.09;
    }

    return tmp_omega;
}


/**
 * @brief 四舍五入
 * @param f 输入的值
 * @return 四舍五入后的值
*/
int Omega::round2int(float f){
    int int_f = f;
    float d = f - int_f;
    if(d >= 0.5) return int_f;
    else if(d < 0 && d >- 0.5) return int_f;
    else if(d > 0 && d < 0.5) return int_f;
    else return int_f - 1;
}


/**
 * @brief 顺逆时针
*/
void Omega::JudgeFanRotation(float omega){
    clockwise_cnt_ = (omega > 0)? clockwise_cnt_ + 1 : clockwise_cnt_ - 1;
    energy_rotation_direction_ = (clockwise_cnt_ > 0)? 1 : -1;
}


/**
 * @brief 设置a w phi三个参数
*/
void Omega::set_a_w_phi(float a, float w, float phi){
    a_ = a;
    w_ = w;
    phi_ = phi;
}


/**
 * @brief 设置时间
 * @param time_t 外部输入当前时间
*/
void Omega::set_time(double time_t){
    time_gap_ += (time_start_==0) ? 0.0 : time_t - time_start_;
    time_start_ = time_t;
    t_list_.push_back(time_gap_);
    dt_ = t_list_.back() - t_list_[t_list_.size() - 2];
}

/**
 * @brief 设置time_series
*/
void Omega::change_time_series(){
    if(start_){
        time_series_.push_back(t_list_[t_list_.size() - 1 - differ_step_ / 2]);
    }
}


/**
 * @brief 根据官方的公式求角速度
*/
float Omega::IdealOmega(float & t_){
    return a_ * std::sin(w_ * t_ + phi_) + 2.09 + a_;
}


/**
 * @brief 根据公式积分求提前量
*/
float Omega::IdealRad(float t1, float t2){
    return (-a_ / w_ ) * (std::cos(w_ * t2 + phi_) - std::cos(w_ * t2 + phi_)) + (2.09 - a_) * (t2 - t1);
}


/**
 * @brief 获取预测角速度
*/
float Omega::get_omega(){
    float wt_ = IdealOmega(time_series_.back());
    return wt_ * energy_rotation_direction_;
}


/**
 * @brief 获取预测角度
 * @param latency 根据姿态解算和弹道解算求出来的时间
*/
float Omega::get_rad(float latency){
    float predict_rad = IdealRad(t_list_.back(), t_list_.back() + latency);
    return predict_rad * energy_rotation_direction_;
}

/**
 * @brief 找到波峰或者波谷来确定相位
 * @return 是否找到
*/
bool Omega::FindWavePeak(){
    if(filter_omega_.size() > 15){
        std::vector<float>cut_filter_omega(filter_omega_.end()-8, filter_omega_.end());
        std::vector<float>cut_time_series(time_series_.end() - 8, time_series_.end());
        Eigen::MatrixXd rate = LeastSquare(cut_time_series, cut_filter_omega, 1);
        fit_cnt_ = (rate(0, 0) > 0) ? fit_cnt_ + 1 : fit_cnt_ -1;
        if(fit_cnt_ > 5){
            if(fabs(filter_omega_.back()) > max_omega_){
                change_cnt_ = 0;
                max_omega_ = filter_omega_.back();
                phi_ = CV_2PI / 2 - w_ * time_series_.back();
                while(phi_ < -CV_PI || phi_ > CV_PI){
                    phi_ += 2 * CV_PI;
                }
            }else{
                change_cnt_++;
            }
        }else if(fit_cnt_ < -5){
            if(fabs(filter_omega_.back()) < min_omega_){
                change_cnt_ = 0;
                min_omega_ = filter_omega_.back();
                phi_ = -CV_PI / 2 - w_ * time_series_.back();
                while(phi_ < -CV_PI ||  phi_ > CV_PI){
                    phi_ += 2 * CV_PI;
                }
            }else{
                change_cnt_++;
            }
        }else{
            return false;
        }

        if(change_cnt_ > 5){
            if(fit_cnt_ >= 0){
                std::cout<<"--- get peak omega :"<<max_omega_<<"   init phi :"<<phi_<<std::endl;
            }else{
                std::cout<<"--- get valley omega :"<<min_omega_<<"   init phi :"<<phi_<<std::endl;
            }
            return true;
        }else{
            return false;
        }
    }else return false;
}


/**
 * @brief 最小二乘拟合
 * @param x 输入值
 * @param y 输入值
 * @param N 数量
 * @return 拟合的权重
*/
Eigen::MatrixXd Omega::LeastSquare(std::vector<float> x, std::vector<float> y, int N){
    Eigen::MatrixXd A(x.size(), N+1);
    Eigen::MatrixXd B(y.size(), 1);
    Eigen::MatrixXd W;

    for(unsigned int i=0; i < x.size(); i++){
        for(int n = N, dex = 0; n >= 1; --n, ++dex){
            A(i, N) = 1;
            B(i, 0) = y[i];
        }
    }
    W = (A.transpose() * A).inverse() * A.transpose() * B;
    return W;
}


/**
 * @brief 获取时间间隔
*/
float Omega::get_time_gap(){
    return time_series_.back() - time_series_[st_];
}

/**
 * @brief 设置滤波后的值
*/
void Omega::set_filter(float d){
    filter_omega_.push_back(energy_rotation_direction_ * d);
}


/**
 * @brief 获取预测误差
*/
float Omega::get_err(){
    return get_omega() - filter_omega_.back();
}


/**
 * @brief 改变st_的值
*/
void Omega::change_st(){
    st_ = (filter_omega_.size() - 80)? filter_omega_.size()-80 : 0;
}