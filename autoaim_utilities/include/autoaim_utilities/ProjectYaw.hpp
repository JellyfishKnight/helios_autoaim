#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <angles/angles.h>
#include <vector>
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

    void caculate_armor_yaw(const Armor &armor, cv::Mat &r_mat, cv::Mat tvec, 
                            geometry_msgs::msg::TransformStamped ts);

    void get_transform_info(geometry_msgs::msg::TransformStamped ts);

    void draw_projection_points(cv::Mat& image);
private:
    double diff_function(double yaw);

    double phi_optimization(double left, double right, double eps);

    void get_rotation_matrix(double yaw, cv::Mat& rotation_mat) const;

    std::vector<cv::Point2f> projected_points_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat odom2cam_r_;
    std::vector<cv::Point2f> image_points_;
    cv::Mat tvec_;
    std::vector<cv::Point3f> object_points_;

    std::vector<cv::Point3f> small_armor_points_;
    std::vector<cv::Point3f> large_armor_points_;
    std::vector<cv::Point3f> energy_armor_points_;
    // The pitch and roll of armor are fixed for target
    double roll_ = 0, pitch_ = angles::from_degrees(15);

    rclcpp::Logger logger_ = rclcpp::get_logger("ProjectYaw");
};


};