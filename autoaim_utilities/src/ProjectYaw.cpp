#include "ProjectYaw.hpp"
#include "Armor.hpp"
#include "PnPSolver.hpp"
#include <angles/angles.h>
#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/types.h>
#include <cmath>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <math.h>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

namespace helios_cv {

ProjectYaw* ProjectYaw::pthis_;

ProjectYaw::ProjectYaw(const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs) :
    camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
    dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone()) {
    // Unit: m
    double half_y{};
    double half_z{};
    // Start from bottom left in clockwise order
    // Model coordinate: x forward, y left, z up
    // Small armor
    // Unit: m
    half_y = small_armor_width / 2.0 / 1000.0;
    half_z = small_armor_height / 2.0 / 1000.0;
    small_armor_points_.emplace_back(cv::Point3f(0, half_y, -half_z));
    small_armor_points_.emplace_back(cv::Point3f(0, half_y, half_z));
    small_armor_points_.emplace_back(cv::Point3f(0, -half_y, half_z));
    small_armor_points_.emplace_back(cv::Point3f(0, -half_y, -half_z));
    // Large armor
    half_y = large_armor_width / 2.0 / 1000.0;
    half_z = large_armor_height / 2.0 / 1000.0;
    large_armor_points_.emplace_back(cv::Point3f(0, half_y, -half_z));
    large_armor_points_.emplace_back(cv::Point3f(0, half_y, half_z));
    large_armor_points_.emplace_back(cv::Point3f(0, -half_y, half_z));
    large_armor_points_.emplace_back(cv::Point3f(0, -half_y, -half_z));
    // Energy armor
    half_y = energy_armor_width / 2.0 / 1000.0;
    half_z = energy_armor_height / 2.0 / 1000.0;
    energy_armor_points_.emplace_back(cv::Point3f(0, half_y, -half_z));
    energy_armor_points_.emplace_back(cv::Point3f(0, half_y, half_z));
    energy_armor_points_.emplace_back(cv::Point3f(0, -half_y, half_z));
    energy_armor_points_.emplace_back(cv::Point3f(0, -half_y, -half_z));

    pthis_ = this;
}

ProjectYaw::~ProjectYaw() {}

cv::Mat ProjectYaw::get_transform_info(geometry_msgs::msg::TransformStamped ts) {
    // turn quaternion into rotation matrix 
    // for more see paper: https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    double x = ts.transform.rotation.x,  y = ts.transform.rotation.y,  z = ts.transform.rotation.z, w = ts.transform.rotation.w;
    return (cv::Mat_<double>(3, 3) << 
            1 - 2 * (z * z + y * y), 2 * (y * x - z * w), 2 * (x * z + y * w),
            2 * (y * x + z * w), 1 - 2 * (x * x + z * z), 2 * (z * y - x * w),
            2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (y * y + x * x));
}

double ProjectYaw::diff_function(double yaw) {
    // Caculate rotation matrix
    cv::Mat rotation_matrix;
    get_rotation_matrix(yaw, rotation_matrix);
    rotation_matrix = odom2cam_r_ * rotation_matrix;
    // Turn rotation matrix into rotation vector
    cv::Mat rvec;
    cv::Rodrigues(rotation_matrix, rvec);
    // caculate projection points
    cv::projectPoints(object_points_, rvec, tvec_, camera_matrix_, dist_coeffs_, projected_points_);
    // Caculate the difference between projected points and image points
    double diff = 0;
    for (int i = 0; i < 4; i++) {
        diff += sqrt(pow(projected_points_[i].x - image_points_[i].x, 2) + pow(projected_points_[i].y - image_points_[i].y, 2));
    }
    diff /= 4;
    return diff;
}

void ProjectYaw::draw_projection_points(cv::Mat& image) {
    if (projected_points_.empty()) {
        RCLCPP_DEBUG(logger_, "empty projection");
        return;
    }
    for (int i = 0; i < 4; i++) {
        cv::circle(image, projected_points_[i], i + 1, cv::Scalar(0, 0, 255), -1);
        // cv::circle(image, image_points_[i], i + 1, cv::Scalar(255, 0, 0), 2);
    }
    cv::line(image, projected_points_[0], projected_points_[2], cv::Scalar(255, 255, 0), 2);
    cv::line(image, projected_points_[1], projected_points_[3], cv::Scalar(255, 255, 0), 2);
    cv::line(image, projected_points_[0], projected_points_[1], cv::Scalar(255, 255, 0), 2);
    cv::line(image, projected_points_[1], projected_points_[2], cv::Scalar(255, 255, 0), 2);
    cv::line(image, projected_points_[2], projected_points_[3], cv::Scalar(255, 255, 0), 2);
    cv::line(image, projected_points_[3], projected_points_[0], cv::Scalar(255, 255, 0), 2);
    projected_points_.clear();
}

double ProjectYaw::phi_optimization(double left, double right, double eps) {
    // Make fibonacci sequence
    std::vector<double> fn(2);
    std::vector<double> fib_seq;
    fn[0] = 1, fn[1] = 1;
    double f_lambda = 0, f_mu = 0;
    int fib_cnt;
    for (fib_cnt = 2; fn[fib_cnt - 1] < 1.0 / eps; fib_cnt++) {
        fn.push_back(fn[fib_cnt - 2] + fn[fib_cnt - 1]);
    }
    fib_cnt--;
    double lambda = left + (fn[fib_cnt - 2] / fn[fib_cnt]) * (right - left);
    double mu = left + (fn[fib_cnt - 1] / fn[fib_cnt]) * (right - left);
    f_lambda = diff_function(lambda);
    f_mu = diff_function(mu);
    for (int i = 0; i < fib_cnt - 2; i++) {
        if (f_lambda < f_mu) {
            right = mu;
            mu = lambda;
            f_mu = f_lambda;
            lambda = left + (fn[fib_cnt - i - 3] / fn[fib_cnt - i - 1]) * (right - left);
            f_lambda = diff_function(lambda);
        } else {
            left = lambda;
            lambda = mu;
            f_lambda = f_mu;
            mu = left + (fn[fib_cnt - i - 2] / fn[fib_cnt - i - 1]) * (right - left);
            f_mu = diff_function(mu);
        }
    }
    f_mu = diff_function(mu);
    if (f_lambda < f_mu) {
        right = mu;
    } else {
        left = lambda;
    }
    return 0.5 * (left + right);
}

void ProjectYaw::get_rotation_matrix(double yaw, cv::Mat& rotation_mat) const {
    // caculate rotation matrix
    // for more see paper: https://danceswithcode.net/engineeringnotes/rotations_in_3d/rotations_in_3d_part1.html
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
               1,       0,               0,
               0,       std::cos(roll_), -std::sin(roll_),
               0,       std::sin(roll_), std::cos(roll_));
     
    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3,3) <<
               std::cos(pitch_),    0,      std::sin(pitch_),
               0,                   1,      0,
               -std::sin(pitch_),   0,      std::cos(pitch_));
     
    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
               std::cos(yaw),    -std::sin(yaw),      0,
               std::sin(yaw),    std::cos(yaw),       0,
               0,                0,                   1);
    rotation_mat = R_z * R_y * R_x;
}

void ProjectYaw::caculate_armor_yaw(const Armor &armor, cv::Mat &r_mat, cv::Mat tvec) {
    double yaw = -M_PI;
    tvec_ = tvec;
    // Fill in image points
    cv::Point2f point;
    image_points_.clear();
    point.x = armor.left_light.bottom.x;
    point.y = armor.left_light.bottom.y;
    image_points_.emplace_back(point);
    point.x = armor.left_light.top.x;
    point.y = armor.left_light.top.y;
    image_points_.emplace_back(point);
    point.x = armor.right_light.top.x;
    point.y = armor.right_light.top.y;
    image_points_.emplace_back(point);
    point.x = armor.right_light.bottom.x;
    point.y = armor.right_light.bottom.y;
    image_points_.emplace_back(point);
    armor_angle_ = armor.angle;
    // Fill in object points
    if (armor.type == ArmorType::SMALL) {
        object_points_ = small_armor_points_;
    } else if (armor.type == ArmorType::LARGE) {
        object_points_ = large_armor_points_;
    } else if (armor.type == ArmorType::ENERGY) {
        object_points_ = energy_armor_points_;
    }
    // Choose pitch value
    if (armor.number == "outpost") {
        pitch_ = angles::from_degrees(-15.0);
    } else {
        pitch_ = angles::from_degrees(15.0);
    }
    // Get min diff yaw
    ceres::Problem problem;
    ceres::CostFunction* costfunctor = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(costfunctor, nullptr, &yaw);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 30;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    diff_function(yaw);
    // RCLCPP_INFO(logger_, "yaw: %f", yaw);
    // Caculate rotation matrix
    get_rotation_matrix(yaw, r_mat);
    r_mat = odom2cam_r_ * r_mat;
}

} // namespace helios_cv