#include "ProjectYaw.hpp"
#include "Armor.hpp"
#include "PnPSolver.hpp"
#include <cmath>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

namespace helios_cv {

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
}

ProjectYaw::~ProjectYaw() {}

double ProjectYaw::diff_function(double yaw) {
    // Caculate rotation matrix
    cv::Mat_<double> rotation_matrix = (cv::Mat_<double>(3, 3) <<
                                        cos(yaw) * cos(pitch_), -sin(yaw) * cos(roll_) + cos(yaw) * sin(pitch_) * sin(roll_), sin(yaw) * sin(roll_) + cos(yaw) * sin(pitch_) * cos(roll_),
                                        sin(yaw) * cos(pitch_), cos(yaw) * cos(roll_) + sin(yaw) * sin(pitch_) * sin(roll_), -cos(yaw) * sin(roll_) + sin(yaw) * sin(pitch_) * cos(roll_),
                                        -sin(pitch_), cos(pitch_) * sin(roll_), cos(pitch_) * cos(roll_));
    rotation_matrix = odom2cam_ * rotation_matrix;
    // Turn rotation matrix into rotation vector
    cv::Mat rvec;
    cv::Rodrigues(rotation_matrix, rvec);
    // caculate projection points
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points_, rvec, cv::Mat::zeros(3, 1, CV_64F), camera_matrix_, dist_coeffs_, projected_points);
    // Caculate the difference between projected points and image points
    double diff = 0;
    for (int i = 0; i < 4; i++) {
        diff += sqrt(pow(projected_points[i].x - image_points_[i].x, 2) + pow(projected_points[i].y - image_points_[i].y, 2));
    }
    diff /= 4;
    return diff;
}

double ProjectYaw::phi_optimization(double left, double right, double eps) {
    const double phi = (1 + sqrt(5)) / 2; // φ 的值
    double x1 = right - (right - left) / phi;
    double x2 = left + (right - left) / phi;
    double f1 = diff_function(x1);
    double f2 = diff_function(x2);
    int iterate_cnt = 0;
    while (fabs(right - left) > eps) {
        iterate_cnt++;
        if (f1 < f2) {
            right = x2;
            x2 = x1;
            f2 = f1;
            x1 = right - (right - left) / phi;
            f1 = diff_function(x1);
        } else {
            left = x1;
            x1 = x2;
            f1 = f2;
            x2 = left + (right - left) / phi;
            f2 = diff_function(x2);
        }
        if (iterate_cnt > 10) {
            RCLCPP_ERROR(logger_, "Phi optimization failed to converge");
            return (left + right) / 2;
        }
    }
    return (left + right) / 2;
}

void ProjectYaw::caculate_armor_yaw(const Armor &armor, cv::Mat &r_mat, cv::Mat &t_vec, cv::Mat& odom2cam) {
    odom2cam_ = odom2cam.clone();
    double yaw;
    cv::Mat rvec, tvec;
    // Fill in object points
    if (armor.type == ArmorType::SMALL) {
        object_points_ = small_armor_points_;
    } else if (armor.type == ArmorType::LARGE) {
        object_points_ = large_armor_points_;
    } else if (armor.type == ArmorType::ENERGY) {
        object_points_ = energy_armor_points_;
    }
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    // Get yaw in about -45 degree to 45 degree
    yaw = phi_optimization(-M_PI_4, -M_PI_4, 1e-2);
    // Caculate rotation matrix
    r_mat = (cv::Mat_<double>(3, 3) <<
        cos(yaw) * cos(pitch_), -sin(yaw) * cos(roll_) + cos(yaw) * sin(pitch_) * sin(roll_), sin(yaw) * sin(roll_) + cos(yaw) * sin(pitch_) * cos(roll_),
        sin(yaw) * cos(pitch_), cos(yaw) * cos(roll_) + sin(yaw) * sin(pitch_) * sin(roll_), -cos(yaw) * sin(roll_) + sin(yaw) * sin(pitch_) * cos(roll_),
        -sin(pitch_), cos(pitch_) * sin(roll_), cos(pitch_) * cos(roll_));        
    // Get translation vector
    t_vec = tvec;
}

} // namespace helios_cv