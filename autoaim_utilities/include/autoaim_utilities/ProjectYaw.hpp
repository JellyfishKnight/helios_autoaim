#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "Armor.hpp"

namespace helios_cv {

class ProjectYaw {
public:
    explicit ProjectYaw(const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs);

    ~ProjectYaw();

    double caculate_armor_yaw(const Armor& armor);
private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};


};