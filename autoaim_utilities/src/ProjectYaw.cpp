#include "ProjectYaw.hpp"

namespace helios_cv {
ProjectYaw::ProjectYaw(const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs) :
    camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
    dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone()) {}

ProjectYaw::~ProjectYaw() {

}

double ProjectYaw::caculate_armor_yaw(const Armor& armor) {
    double yaw;


    return yaw;
}

} // namespace helios_cv