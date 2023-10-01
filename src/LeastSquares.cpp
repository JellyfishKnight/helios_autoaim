// created by liuhan on 2023/9/29
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "LeastSquares.hpp"

namespace helios_cv {

void LeastSquares::solve_problem(Omega omega) {
    refresh = omega.refresh;
    if(refresh == true){
        CeresRefresh();
    }
    float omega_ = omega.omega;
    filter_omega.push_back(omega_);

    a = omega.a;
    w = omega.w;
    phi = omega.phi;
    t.push_back(omega.t);
    isSolve = omega.solve;
    st = (omega.st > 0) ? omega.st: 0;
    if (isSolve == true) {
        for (long unsigned int i = st; i < filter_omega.size(); i += 2) {
            ceres::CostFunction* const_func = 
                                    new ceres::AutoDiffCostFunction<SinResidual, 1, 1, 1, 1>(new SinResidual(t[i], filter_omega[i]));
            problem->AddResidualBlock(const_func, NULL, &a, &w, &phi);
        }
        //范围都是官方给的
        problem->SetParameterLowerBound(&a,0,0.78);//0 0.78
        problem->SetParameterUpperBound(&a,0,1.045);//0 1.045
        problem->SetParameterLowerBound(&w,0,1.884);//0 1.884
        problem->SetParameterUpperBound(&w,0,2.0);//0 2.0
        problem->SetParameterLowerBound(&phi,0,-3.14159265357989);//CV_PI
        problem->SetParameterUpperBound(&phi,0,3.14159265357989);

        ceres::Solver::Options options;
        options.max_num_iterations = 50;
    
        options.linear_solver_type = ceres::DENSE_QR;//DENDE——OR
        options.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        ceres::Solve(options, problem.get(), &summary);

        if (a<0.780){
            a = 0.780;
        } else if (a > 1.045) {
            a = 1.045;
        }
        if (w < 0) {
            w=fabs(w);
        }
        if (w < 1.884) {
            w = 1.884;
        } else if (w > 2.0) {
            w = 2.0;
        }
    }else{
        return;
    }
}

Omega LeastSquares::get_result() const {
    Omega omega_;
    omega_.a = a;
    omega_.w = w;
    omega_.phi = phi;
    return omega_;
}

void LeastSquares::CeresRefresh() {
    problem.reset(new ceres::Problem);
    filter_omega.clear();
    t.clear();
    phi = 3.14159;
    w = 1.942;   //1.884~2.0
    a = 0.9125;  //0.78~1.045
    isSolve = false;
}


} // namespace helios_cv