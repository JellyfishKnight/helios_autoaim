#include "TraditionalArmorDetector.hpp"
#include <rclcpp/logging.hpp>

namespace helios_cv {
    TraditionalArmorDetector::TraditionalArmorDetector(helios_autoaim::Params::Detector::ArmorDetector detector_params) {
        params_ = detector_params;
        pnp_solver_ = nullptr;
        number_classifier_ = nullptr;
        number_classifier_ = std::make_shared<NumberClassifier>(
            params_.number_classifier.module_path, params_.number_classifier.label_path,
            params_.number_classifier.threshold);
    }
    
    void TraditionalArmorDetector::set_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr cam_info) {
        cam_info_ = cam_info;
        cam_center_ = cv::Point2f(cam_info_->k[2], cam_info_->k[5]);
        pnp_solver_ = std::make_shared<PnPSolver>(cam_info->k, cam_info->d);
    }

    bool TraditionalArmorDetector::init_detector(helios_autoaim::Params::Detector detector_param) {
        return true;
    }

    helios_rs_interfaces::msg::Armors TraditionalArmorDetector::detect_targets(sensor_msgs::msg::Image::SharedPtr image) {
        if (pnp_solver_ == nullptr || number_classifier_ == nullptr) {
            RCLCPP_WARN(logger_, "Detector not initialized");
            return helios_rs_interfaces::msg::Armors();
        }
        // convert image
        auto img = cv_bridge::toCvShare(image, "rgb8")->image;
        // preprocess
        binary_img_ = preprocessImage(img);
        lights_ = findLights(img, binary_img_);
        armors_ = matchLights(lights_);
        if (!armors_.empty()) {
            number_classifier_->extractNumbers(img, armors_);
            number_classifier_->classify(armors_);
        }
        // convert armors into interfaces
        ///TODO: convert armors into interfaces
    }

    cv::Mat& TraditionalArmorDetector::draw_results() {

    }

    void TraditionalArmorDetector::set_params(helios_autoaim::Params::Detector detector_params) {
        
    }

    cv::Mat TraditionalArmorDetector::preprocessImage(const cv::Mat & input) {

    }

    std::vector<Light> TraditionalArmorDetector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img) {

    }
    
    std::vector<Armor> TraditionalArmorDetector::matchLights(const std::vector<Light> & lights) {

    }

    bool TraditionalArmorDetector::isLight(const Light & possible_light) {

    }
    
    bool TraditionalArmorDetector::containLight(
        const Light & light_1, const Light & light_2, const std::vector<Light> & lights) {

    }
    
    ArmorType TraditionalArmorDetector::isArmor(const Light & light_1, const Light & light_2) {

    }


} // namespace helios_cv