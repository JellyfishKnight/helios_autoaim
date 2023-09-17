#include "HeliosAutoAim.hpp"
#include "ArmorPredictor.hpp"
#include "EnergyPredictor.hpp"
#include "TraditionalArmorDetector.hpp"
#include "TraditionalEnergyDetector.hpp"
#include "helios_autoaim_parameters.hpp"
#include <cmath>
#include <functional>
#include <helios_rs_interfaces/msg/detail/send_data__struct.hpp>
#include <image_transport/publisher.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <utility>
#include <vector>

namespace helios_cv {

HeliosAutoAim::HeliosAutoAim(const rclcpp::NodeOptions& options) : 
    Node("helios_autoaim", options) {
    // use an incorrect way to make param library work
    ///TODO: need improve
    this_node_ = this;
    std::shared_ptr<rclcpp::Node> temp(this_node_);
    // update params
    param_listener_ = std::make_shared<helios_autoaim::ParamListener>(temp);
    params_ = param_listener_->get_params();
    state_ = State::UNCONFIGURED;
    transition_ = Transition::NONE;
    // state machine
    while (rclcpp::ok()) {
        // refresh parameters if there is any change
        if (param_listener_->is_old(params_)) {
            params_ = param_listener_->get_params();
            transition_ = static_cast<Transition>(params_.transition);
        }
        // finalize
        if (transition_ == Transition::SHUTDOWN) {
            state_ = on_shutdown();
            break;
        }
        // error handling
        if (state_ == State::ERROR) {
            RCLCPP_WARN(logger_, "Handling error");
            state_ = on_error();
            if (state_ == State::ERROR) {
                RCLCPP_ERROR(logger_, "Failed to handle error");
                transition_ = Transition::SHUTDOWN;
            }
        // configure
        } else if (transition_ == Transition::CONFIGURE && state_ == State::UNCONFIGURED) {
            state_ = on_configure();
            if (state_ == State::INACTIVE) {
                transition_ = Transition::NONE;
                RCLCPP_DEBUG(logger_, "Autoaim configure success");
            } else {
                RCLCPP_ERROR(logger_, "Failed to configure");
                state_ = State::ERROR;
            }
        // activate
        } else if (transition_ == Transition::ACTIVATE && state_ == State::INACTIVE) {
            state_ = on_activate();
            if (state_ == State::ACTIVE) {
                transition_ = Transition::NONE;
                RCLCPP_DEBUG(logger_, "Autoaim activate success");
            } else {
                RCLCPP_ERROR(logger_, "Failed to activate");
                state_ = State::ERROR;
            }
        // deactivate
        } else if (transition_ == Transition::DEACTIVATE && state_ == State::ACTIVE) {
            state_ = on_deactivate();
            if (state_ == State::INACTIVE) {
                transition_ = Transition::NONE;
                RCLCPP_DEBUG(logger_, "Autoaim deactivate success");
            } else {
                RCLCPP_ERROR(logger_, "Failed to deactivate");
                state_ = State::ERROR;
            }
        // clean up
        } else if (transition_ == Transition::CLEANUP && state_ == State::INACTIVE) {
            state_ = on_cleanup();
            if (state_ == State::UNCONFIGURED) {
                transition_ = Transition::NONE;
                RCLCPP_DEBUG(logger_, "Autoaim cleanup success");
            } else {
                RCLCPP_ERROR(logger_, "Failed to cleanup");
                state_ = State::ERROR;
            }
        }
        // update state
        param_listener_->update(std::vector<rclcpp::Parameter>{
            rclcpp::Parameter("state", static_cast<int>(state_))});
    }
    RCLCPP_INFO(logger_, "Autoaim shutdown success");
}

State HeliosAutoAim::on_configure() {
    // create detector and predictor
    if (params_.armor_autoaim) {
        if (params_.use_traditional) {
            detector_ = std::make_shared<TraditionalArmorDetector>(params_.detector.armor_detector);
        } else {
            
        }
        predictor_ = std::make_shared<ArmorPredictor>(params_.predictor.armor_predictor);
    } else {
        if (params_.use_traditional) {
            detector_ = std::make_shared<TraditionalEnergyDetector>(params_.detector.energy_detector);
        } else {

        }
        predictor_ = std::make_shared<EnergyPredictor>(params_.predictor.energy_predictor);
    }
    // create fire controller
    fire_controller_ = std::make_shared<FireController>(params_.fire_controller);
    // avoid nullptr
    cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    return State::INACTIVE;
}

State HeliosAutoAim::on_activate() {
    // activate detector and predictor
    detector_->init_detector(params_.detector);
    predictor_->init_predictor(params_.predictor);
    fire_controller_->init_fire_controller(params_.fire_controller);
    // create debug publishers    
    if (params_.debug) {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "armor_marker", rclcpp::SensorDataQoS());
        binary_img_pub_ = std::make_shared<image_transport::Publisher>();
        number_img_pub_ = std::make_shared<image_transport::Publisher>();
        result_img_pub_ = std::make_shared<image_transport::Publisher>();
    }
    // Create publishers and subscribers
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info) {
            RCLCPP_INFO_ONCE(logger_, "Camera info received");
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*cam_info);
            detector_->set_cam_info(std::move(cam_info_));
            // receive cam info only once
            cam_info_sub_.reset();
        });
    target_data_pub_ = this->create_publisher<helios_rs_interfaces::msg::SendData>(
        "autoaim_cmd", rclcpp::SensorDataQoS());
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(), std::bind(&HeliosAutoAim::image_callback, this, std::placeholders::_1));
    return State::ACTIVE;
}

State HeliosAutoAim::on_deactivate() {
    cam_info_.reset();
    target_data_pub_.reset();
    image_sub_.reset();
    binary_img_pub_.reset();
    number_img_pub_.reset();
    result_img_pub_.reset();
    return State::INACTIVE;
}

State HeliosAutoAim::on_cleanup() {
    return State::UNCONFIGURED;
}

State HeliosAutoAim::on_shutdown() {
    return State::FINALIZED;
}

State HeliosAutoAim::on_error() {
    return State::ERROR;
}

void HeliosAutoAim::image_callback(sensor_msgs::msg::Image::SharedPtr msg) {
    // check if params are updated
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        detector_->set_params(params_.detector);
        predictor_->set_params(params_.predictor);
        fire_controller_->set_params(params_.fire_controller);
    }
    auto armors = detector_->detect_targets(std::move(msg));
    if (armors.armors.empty()) {
        RCLCPP_DEBUG(logger_, "No armor detecter");
        return ;
    }
    
}

} // namespace helios_cv

// register node to component
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::HeliosAutoAim);