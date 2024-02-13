#include "PnPSolver.hpp"
#include "Armor.hpp"

#include <rclcpp/logging.hpp>
#include <vector>

namespace helios_cv {

PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  double half_y{};
  double half_z{};

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y le`ft, z up
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
}

bool PnPSolver::solve_pnp(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
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
  } else {
    return false;
  }
  
  return cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

bool PnPSolver::solve_pose(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec) {
  if (solve_pnp(armor, rvec, tvec)) {
    return true;
  } else {
    return false;
  }
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

void ArmorProjectYaw::update_transform_info(const cv::Quatd& odom2cam_r, const cv::Quatd& cam2odom_r) {
  odom2cam_r_ = odom2cam_r.toRotMat3x3();
  cam2odom_r_ = cam2odom_r.toRotMat3x3();
}


double ArmorProjectYaw::diff_function(double yaw) {
    // Caculate rotation matrix
    cv::Mat rotation_matrix;
    get_rotation_matrix(yaw, rotation_matrix);
    rotation_matrix = odom2cam_r_ * rotation_matrix;
    // Turn rotation matrix into rotation vector
    // caculate projection points
    cv::projectPoints(object_points_, rotation_matrix, tvec_, camera_matrix_, dist_coeffs_, projected_points_);
    // Caculate the difference between projected points and image points by angle between vectors
    double diff = 0;
    Eigen::Vector3d image_vector[2], projected_vector[2];
    image_vector[0] << image_armor_points[3].x - image_armor_points[0].x, image_armor_points[3].y - image_armor_points[0].y, 1;
    image_vector[1] << image_armor_points[1].x - image_armor_points[2].x, image_armor_points[1].y - image_armor_points[2].y, 1;
    projected_vector[0] << projected_points_[3].x - projected_points_[0].x, projected_points_[3].y - projected_points_[0].y, 1;
    projected_vector[1] << projected_points_[1].x - projected_points_[0].x, projected_points_[1].y - projected_points_[0].y, 1;
    Eigen::Vector3d image_cross = image_vector[0].cross(image_vector[1]);
    Eigen::Vector3d projected_cross = projected_vector[0].cross(projected_vector[1]);
    double raw_angle = image_vector[0].dot(image_vector[1]) / (image_vector[0].norm() * image_vector[1].norm());
    double project_angle;
    if (image_cross.dot(projected_cross) >= 0) {
        project_angle = projected_vector[0].dot(projected_vector[1]) / (projected_vector[0].norm() * projected_vector[1].norm());
    } else {
        project_angle = -projected_vector[0].dot(projected_vector[1]) / (projected_vector[0].norm() * projected_vector[1].norm());
    }
    return raw_angle - project_angle;
}

void ArmorProjectYaw::draw_projection_points(cv::Mat& image) {
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

double ArmorProjectYaw::phi_optimization(double left, double right, double eps) {
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

void ArmorProjectYaw::get_rotation_matrix(double yaw, cv::Mat& rotation_mat) const {
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

bool ArmorProjectYaw::solve_pose(const Armor &armor, cv::Mat &rvec, cv::Mat &tvec) {
  /*Solve PnP First To Get Position Info*/
  // Fill in image points
  if (!use_projection) {
    return solve_pnp(armor, rvec, tvec);
  } else {
    if (!solve_pnp(armor, rvec, tvec)) {
      return false;
    }
  }
  if (!is_transform_info_updated_) {
    RCLCPP_WARN(logger_, "PnP Solve done, but transform info not updated, skipping");
    return true;
  }
  /*Start Reprojection*/
  double yaw = 0;
  tvec_ = tvec;
  armor_angle_ = armor.angle;
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
  options.max_num_iterations = 100;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // Get draw points
  diff_function(yaw);
  // Caculate rotation matrix
  get_rotation_matrix(yaw, rvec);
  rvec = odom2cam_r_ * rvec;
  return true;
}

} // namespace helios_cv