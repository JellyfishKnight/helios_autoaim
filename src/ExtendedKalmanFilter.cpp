#include "ExtendedKalmanFilter.hpp"

namespace helios_cv {

ExtendedKalmanFilter::ExtendedKalmanFilter(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& F,
                         const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& H,
                         const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& Jf,
                         const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& Jh,
                         const std::function<Eigen::MatrixXd()>& UpdateQ, 
                         const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& UpdateR,
                         const Eigen::MatrixXd& P) : f(F), h(H), jf(Jf), jh(Jh), update_Q(UpdateQ), update_R(UpdateR), 
                                                     P_post_(P) 
{
    state_pre_ = Eigen::VectorXd::Zero(P.rows());
    state_post_ = Eigen::VectorXd::Zero(P.rows());
}

Eigen::MatrixXd ExtendedKalmanFilter::Predict() {
    F = jf(state_post_);
    Q = update_Q();
    state_pre_ = f(state_post_);
    P_pre_ = F * P_post_ * F.transpose() + Q;
    state_post_ = state_pre_;
    P_post_ = P_pre_;    
    return state_pre_;
}

Eigen::MatrixXd ExtendedKalmanFilter::Correct(const Eigen::VectorXd& z) {
    R = update_R(z);
    H = jh(state_pre_);
    Eigen::MatrixXd S = H * P_pre_ * H.transpose() + R;
    Eigen::MatrixXd K = P_pre_ * H.transpose() * S.inverse();
    Eigen::VectorXd y = z - h(state_pre_);
    state_post_ = state_pre_ + K * y;
    P_post_ = (Eigen::MatrixXd::Identity(P_pre_.rows(), P_pre_.cols()) - K * H) * P_pre_;
    return state_post_;
}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd& state) {
    state_post_ = state;
}

}; // namespace helios_cv