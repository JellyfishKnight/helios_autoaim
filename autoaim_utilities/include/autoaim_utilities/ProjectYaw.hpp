#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <rclcpp/logger.hpp>
#include <angles/angles.h>

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

    double diff_function(double yaw);

    double phi_optimization(double left, double right, double eps);

    double caculate_armor_yaw(const Armor& armor, double& distance_to_image_center);
private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    std::vector<cv::Point2f> image_points_;
    std::vector<cv::Point3f> object_points_;

    std::shared_ptr<PnPSolver> pnp_solver_;  

    rclcpp::Logger logger_ = rclcpp::get_logger("ProjectYaw");
};


};