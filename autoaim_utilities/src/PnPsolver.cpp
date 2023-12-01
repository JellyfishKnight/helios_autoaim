// Copyright 2022 Chen Jun

#include "PnPSolver.hpp"
#include "Armor.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>


namespace helios_cv {

PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs,
  const PnPParams& pnp_solver_params)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone()),
  pnp_solver_params_(pnp_solver_params)
{
  // Unit: m
  double half_y{};
  double half_z{};

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  // Small armor
  // Unit: m
  half_y = pnp_solver_params.small_armor_width / 2.0 / 1000.0;
  half_z = pnp_solver_params.small_armor_height / 2.0 / 1000.0;

  small_armor_points_.emplace_back(cv::Point3f(0, half_y, -half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, half_y, half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -half_y, half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -half_y, -half_z));

  // Large armor
  half_y = pnp_solver_params.large_armor_width / 2.0 / 1000.0;
  half_z = pnp_solver_params.large_armor_height / 2.0 / 1000.0;

  large_armor_points_.emplace_back(cv::Point3f(0, half_y, -half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, half_y, half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -half_y, half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -half_y, -half_z));

  // Energy armor
  half_y = pnp_solver_params.energy_armor_width / 2.0 / 1000.0;
  half_z = pnp_solver_params.energy_armor_height / 2.0 / 1000.0;

  energy_armor_points_.emplace_back(cv::Point3f(0, half_y, -half_z));
  energy_armor_points_.emplace_back(cv::Point3f(0, half_y, half_z));
  energy_armor_points_.emplace_back(cv::Point3f(0, -half_y, half_z));
  energy_armor_points_.emplace_back(cv::Point3f(0, -half_y, -half_z));
}

bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_armor_points;

  // Fill in image points
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  // Solve pnp
  std::vector<cv::Point3f> object_points;
  if (armor.type == ArmorType::SMALL) {
    object_points = small_armor_points_;
  } else if (armor.type == ArmorType::LARGE) {
    object_points = large_armor_points_;
  } else if (armor.type == ArmorType::ENERGY) {
    object_points = energy_armor_points_;
  } else {
    return false;
  }
  
  return cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

} // namespace helios_cv