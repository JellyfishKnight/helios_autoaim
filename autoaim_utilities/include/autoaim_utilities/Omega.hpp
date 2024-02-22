#ifndef OMEGA_HPP
#define OMEGA_HPP
#include"iostream"
#include"cmath"

#include"eigen3/Eigen/Core"
#include"eigen3/Eigen/Dense"

#include"opencv4/opencv2/core.hpp"

class Omega
{
public:
    Omega();
    void refresh();
    void refresh_after_wave();
    void set_theta(double theta);
    bool FindWavePeak();
    double get_omega();
    double get_rad(double latency);
    double get_time_gap();
    void set_time(double time_t);
    void set_filter(double omega_filter);
    void set_a_w_phi(double a, double w, double phi);
    double get_err();
    void change_st();

private:
    double IdealOmega(double &t_);
    double IdealRad(double t1, double t2);
    int round2int(double f);
    double calOmegaNstep(int step, double &total_theta);
    Eigen::MatrixXd LeastSquare(std::vector<double>x, std::vector<double>y, int N);
    void JudgeFanRotation(double omega);
    void change_time_series();

public:

    std::vector<double>angle_;
    std::vector<double>filter_omega_;
    double total_theta_;
    double current_theta_;
    int st_;
    int fit_cnt_;
    double dt_;
    int energy_rotation_direction_;
    bool start_;
    std::vector<double> t_list_, time_series_;
    double phi_, w_, a_;

private:
    int change_cnt_;
    int differ_step_;
    double time_start_, time_fin_, time_gap_;
    int clockwise_cnt_;
    double max_omega_, min_omega_;




};


#endif