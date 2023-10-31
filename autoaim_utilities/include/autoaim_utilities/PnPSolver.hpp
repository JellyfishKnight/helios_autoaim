// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STL
#include <array>
#include <vector>

#include "Armor.hpp"

namespace helios_cv {

typedef struct PnPParams {
  // Small armor
  double small_armor_width;
  double small_armor_height;

  // Large armor
  double large_armor_width;
  double large_armor_height;

  // Energy armor
  double energy_armor_width;
  double energy_armor_height;
}PnPParams;


class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients,
    const PnPParams& pnp_solver_params);

  // Get 3d position
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  // Calculate the distance between armor center and image center
  float calculateDistanceToCenter(const cv::Point2f & image_point);

  void update_params(const PnPParams& pnp_params);

private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  
  PnPParams pnp_solver_params_;

  // Four vertices of armor in 3d
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
  std::vector<cv::Point3f> energy_armor_points_;
};
} // namespace helios_cv

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
