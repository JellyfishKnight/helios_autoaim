#pragma once
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <ceres/ceres.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <angles/angles.h>


#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "Armor.hpp"
#include "PnPSolver.hpp"


namespace helios_cv {

constexpr double small_armor_width = 135.0;
constexpr double small_armor_height = 55.0;
constexpr double large_armor_width = 225.0;
constexpr double large_armor_height = 55.0;
constexpr double energy_armor_width = 380.0;
constexpr double energy_armor_height = 400.0;



class ProjectYaw {
public:
    explicit ProjectYaw(const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs);

    ~ProjectYaw();

    void caculate_armor_yaw(const Armor &armor, cv::Mat &r_mat, cv::Mat tvec);

    cv::Mat get_transform_info(geometry_msgs::msg::TransformStamped ts);

    void draw_projection_points(cv::Mat& image);

    cv::Mat odom2cam_r_;
    cv::Mat cam2odom_r_;
private:
    static ProjectYaw* pthis_;

    struct CostFunctor {
        template<typename T>
        bool operator() (const T* const yaw, T* residual) const {
            // Caculate pose in imu
            Eigen::Matrix<T, 3, 3> rotation_mat, R_x, R_y, R_z;
            R_x << T(1), T(0), T(0),
                    T(0), ceres::cos(T(pthis_->roll_)), -ceres::sin(T(pthis_->roll_)),
                    T(0), ceres::sin(T(pthis_->roll_)), ceres::cos(T(pthis_->roll_));
            R_y << ceres::cos(T(pthis_->pitch_)), T(0), ceres::sin(T(pthis_->pitch_)),
                    T(0), T(1), T(0),
                    -ceres::sin(T(pthis_->pitch_)), T(0), ceres::cos(T(pthis_->pitch_));
            R_z << ceres::cos(*yaw), -ceres::sin(*yaw), T(0),
                    ceres::sin(*yaw), ceres::cos(*yaw), T(0),
                    T(0), T(0), T(1);
            rotation_mat = R_z * R_y * R_x;
            // Convert pose to camera
            Eigen::Matrix<T, 3, 3> odom2cam_r;
            odom2cam_r << T(pthis_->odom2cam_r_.at<double>(0, 0)), T(pthis_->odom2cam_r_.at<double>(0, 1)), T(pthis_->odom2cam_r_.at<double>(0, 2)),
                        T(pthis_->odom2cam_r_.at<double>(1, 0)), T(pthis_->odom2cam_r_.at<double>(1, 1)), T(pthis_->odom2cam_r_.at<double>(1, 2)),
                        T(pthis_->odom2cam_r_.at<double>(2, 0)), T(pthis_->odom2cam_r_.at<double>(2, 1)), T(pthis_->odom2cam_r_.at<double>(2, 2));
            rotation_mat = odom2cam_r * rotation_mat;
            // Project points
            Eigen::Matrix<T, 3, 1> tvec;
            tvec << T(pthis_->tvec_.at<double>(0, 0)), T(pthis_->tvec_.at<double>(1, 0)), T(pthis_->tvec_.at<double>(2, 0));
            std::vector<Eigen::Vector<T, 3>> points;
            for (const auto &point : pthis_->object_points_) {
                Eigen::Matrix<T, 3, 1> point_eigen;
                point_eigen << T(point.x), T(point.y), T(point.z);
                points.emplace_back(rotation_mat * point_eigen + tvec);
            }
            Eigen::Matrix<T, 3, 3> camera_matrix ;
            camera_matrix << T(pthis_->camera_matrix_.at<double>(0, 0)), T(pthis_->camera_matrix_.at<double>(0, 1)), T(pthis_->camera_matrix_.at<double>(0, 2)),
                            T(pthis_->camera_matrix_.at<double>(1, 0)), T(pthis_->camera_matrix_.at<double>(1, 1)), T(pthis_->camera_matrix_.at<double>(1, 2)),
                            T(pthis_->camera_matrix_.at<double>(2, 0)), T(pthis_->camera_matrix_.at<double>(2, 1)), T(pthis_->camera_matrix_.at<double>(2, 2));
            Eigen::Matrix<T, 5, 1> dist_coeffs;
            dist_coeffs << T(pthis_->dist_coeffs_.at<double>(0)), T(pthis_->dist_coeffs_.at<double>(1)), T(pthis_->dist_coeffs_.at<double>(2)), T(pthis_->dist_coeffs_.at<double>(3)), T(pthis_->dist_coeffs_.at<double>(4));
            std::vector<Eigen::Vector<T, 3>> projected_points;
            for (const auto &point : points) {
                Eigen::Vector<T, 3> projected_point;
                projected_point = camera_matrix / point[2] * point;
                projected_points.emplace_back(projected_point);
            }
            // Caculate the difference between projected points and image points
            T diff = T(0);
            for (int i = 0; i < 4; i++) {
                diff += ceres::sqrt(ceres::pow(projected_points[i][0] - T(pthis_->image_points_[i].x), T(2)) + ceres::pow(projected_points[i][1] - T(pthis_->image_points_[i].y), T(2)));
            }
            diff /= T(4);
            residual[0] = diff;
            return true;
        }
    };

    double diff_function(double yaw); 

    [[deprecated]] double phi_optimization(double left, double right, double eps);

    void get_rotation_matrix(double yaw, cv::Mat& rotation_mat) const;

    std::vector<cv::Point2f> projected_points_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<cv::Point2f> image_points_;
    cv::Mat tvec_;
    std::vector<cv::Point3f> object_points_;

    std::vector<cv::Point3f> small_armor_points_;
    std::vector<cv::Point3f> large_armor_points_;
    std::vector<cv::Point3f> energy_armor_points_;
    // The pitch and roll of armor are fixed for target
    double roll_ = 0, pitch_ = angles::from_degrees(15);
    double armor_angle_;

    rclcpp::Logger logger_ = rclcpp::get_logger("ProjectYaw");
};


};