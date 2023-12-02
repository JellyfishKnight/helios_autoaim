#include "ProjectYaw.hpp"
#include "Armor.hpp"
#include "PnPSolver.hpp"
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

void ProjectYaw::get_transform_info(geometry_msgs::msg::TransformStamped ts) {
    double x = ts.transform.rotation.x,  y = ts.transform.rotation.y,  z = ts.transform.rotation.z, w = ts.transform.rotation.w;
    odom2cam_r_ = (cv::Mat_<double>(3, 3) << 
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
    return -diff;
}

void ProjectYaw::draw_projection_points(cv::Mat& image) {
    cv::line(image, projected_points_[0], projected_points_[2], cv::Scalar(255, 0, 0), 2);
    cv::line(image, projected_points_[1], projected_points_[3], cv::Scalar(255, 0, 0), 2);
}

double ProjectYaw::phi_optimization(double left, double right, double eps) {
    const double phi = (1 + std::sqrt(5)) / 2; // φ 的值
    double x1 = right - (right - left) / phi;
    double x2 = left + (right - left) / phi;
    double f1 = diff_function(x1);
    double f2 = diff_function(x2);
    int iterate_cnt = 0;
    while (std::fabs(right - left) > eps) {
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
        if (iterate_cnt > 20) {
            RCLCPP_ERROR(logger_, "Phi optimization failed to converge");
            return (left + right) / 2;
        }
    }
    return (left + right) / 2;
}

void ProjectYaw::get_rotation_matrix(double yaw, cv::Mat& rotation_mat) {
    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
               1,       0,              0,
               0,       std::cos(roll_),-std::sin(roll_),
               0,       std::sin(roll_),std::cos(roll_));
     
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

void ProjectYaw::caculate_armor_yaw(const Armor &armor, cv::Mat &r_mat, cv::Mat tvec, geometry_msgs::msg::TransformStamped ts) {
    double yaw;
    tvec_ = tvec;
    get_transform_info(ts);
    // Fill in image points
    cv::Point2f point;
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
    // Fill in object points
    if (armor.type == ArmorType::SMALL) {
        object_points_ = small_armor_points_;
    } else if (armor.type == ArmorType::LARGE) {
        object_points_ = large_armor_points_;
    } else if (armor.type == ArmorType::ENERGY) {
        object_points_ = energy_armor_points_;
    }
    // // Get yaw in about 0 to 360 degree
    yaw = phi_optimization(0, 2 * M_PI, 1e-2);
    // Caculate rotation matrix
    get_rotation_matrix(yaw, r_mat);
    r_mat = r_mat * odom2cam_r_;
}

} // namespace helios_cv