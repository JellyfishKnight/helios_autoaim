// created by liuhan on 2023/9/29
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

#include "ceres/ceres.h"
#include "ceres/problem.h"
#include "vector"

namespace helios_cv {
/*
 *   设置代价函数  用于优化
 */
struct SinResidual{
    SinResidual(double t, double omega): omega_(omega), t_(t) {}

    template<class T>
    bool operator()(const T* const a, const T* const w, const T* phi, T* residual) const {
        residual[0] = omega_ - (a[0] * sin(w[0] * t_ + phi[0]) + 2.09 - a[0]);
        return true;
    }
private:
    const double omega_;
    const double t_;
};

typedef struct Omega {
    float omega;
    float t;
    int st;
    bool solve;
    float a;
    float w;
    float phi;
    bool refresh;
}Omega;

class LeastSquares {
public:
    LeastSquares() = default;

    ~LeastSquares() = default;

    void solve_problem(Omega omega);

    Omega get_result() const;
private:
    void CeresRefresh();

    std::shared_ptr<ceres::Problem> problem = std::make_shared<ceres::Problem>();

    std::vector<float> filter_omega;
    std::vector<float> t;
    int st;
    double phi = 3.14159;
    double w = 1.942;//1.884~2.0
    double a = 0.9125;//0.78~1.045
    bool isSolve = false;
    bool refresh = false;
    // h1351
};

} // namespace helios_cv