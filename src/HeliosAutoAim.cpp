#include "HeliosAutoAim.hpp"
#include "ArmorPredictor.hpp"
#include "EnergyPredictor.hpp"
#include "TraditionalArmorDetector.hpp"
#include "TraditionalEnergyDetector.hpp"
#include "helios_autoaim_parameters.hpp"
#include <cmath>
#include <functional>
#include <helios_rs_interfaces/msg/detail/target__struct.hpp>
#include <image_transport/publisher.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace helios_cv {

using namespace std::chrono_literals;

HeliosAutoAim::HeliosAutoAim(const rclcpp::NodeOptions& options) : 
    Node("helios_autoaim", options) {
    // update params
    param_listener_ = std::make_shared<helios_autoaim::ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();
    state_ = State::UNCONFIGURED;
    transition_ = Transition::NONE;
    // init activate autoaim 
    while (state_ != State::ACTIVE) {
        if (state_ == State::UNCONFIGURED) {
            state_ = on_configure();
        }
        if (state_ == State::INACTIVE) {
            state_ = on_activate();
        }
        if (state_ == State::ERROR) {
            state_ = on_error();
            if (state_ == State::ERROR) {
                RCLCPP_ERROR(logger_, "Failed to handle error");
                transition_ = Transition::SHUTDOWN;
            }
        }
    }
    timer_ = create_wall_timer(
      10ms, [this]() {
        // refresh parameters if there is any change
        if (param_listener_->is_old(params_)) {
            param_listener_->refresh_dynamic_parameters();
            params_ = param_listener_->get_params();
            transition_ = static_cast<Transition>(params_.transition);
        }
        if (params_.armor_autoaim != last_armor_autoaim_) {
            // change mode, deactivate and clean up first
            while (state_ != State::UNCONFIGURED) {
                if (state_ == State::ACTIVE) {
                    state_ = on_deactivate();
                }
                if (state_ == State::INACTIVE) {
                    state_ = on_cleanup();
                }
                if (state_ == State::ERROR) {
                    state_ = on_error();
                    if (state_ == State::ERROR) {
                        RCLCPP_ERROR(logger_, "Failed to handle error");
                        transition_ = Transition::SHUTDOWN;
                    }
                }
            }
            // reconfigure and activate
            while (state_ != State::ACTIVE) {
                if (state_ == State::UNCONFIGURED) {
                    state_ = on_configure();
                }
                if (state_ == State::INACTIVE) {
                    state_ = on_activate();
                }
                if (state_ == State::ERROR) {
                    state_ = on_error();
                    if (state_ == State::ERROR) {
                        RCLCPP_ERROR(logger_, "Failed to handle error");
                        transition_ = Transition::SHUTDOWN;
                    }
                }
            }
        }
        last_armor_autoaim_ = params_.armor_autoaim;            
    });
}

State HeliosAutoAim::on_configure() {
    // create detector and predictor
    if (params_.armor_autoaim) {
        if (params_.use_traditional) {
            // use traditional way to detect
            detector_ = std::make_shared<TraditionalArmorDetector>(params_.detector.armor_detector);
        } else {
            ///TODO: use net to detect
        }
        predictor_ = std::make_shared<ArmorPredictor>(params_.predictor.armor_predictor);
    } else {
        if (params_.use_traditional) {
            // use traditional way to detect
            detector_ = std::make_shared<TraditionalEnergyDetector>(params_.detector.energy_detector);
        } else {
            ///TODO: use net to detect
        }
        predictor_ = std::make_shared<EnergyPredictor>(params_.predictor.energy_predictor);
    }
    // avoid nullptr
    cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    return State::INACTIVE;
}

State HeliosAutoAim::on_activate() {
    // activate detector and predictor
    detector_->init_detector(params_.detector);
    predictor_->init_predictor(params_.predictor);
    // create debug publishers    
    if (params_.debug) {
        init_markers();
        binary_img_pub_ = std::make_shared<image_transport::Publisher>();
        number_img_pub_ = std::make_shared<image_transport::Publisher>();
        result_img_pub_ = image_transport::create_publisher(this, "/result_img");
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
    // create tf2 utilities
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    /* Create the timer interface before call to waitForTransform,
     to avoid a tf2_ros::CreateTimerInterfaceException exception */
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // 
    target_data_pub_ = this->create_publisher<helios_rs_interfaces::msg::Target>(
        "autoaim_cmd", rclcpp::SensorDataQoS());
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(), std::bind(&HeliosAutoAim::image_callback, this, std::placeholders::_1));
    return State::ACTIVE;
}

State HeliosAutoAim::on_deactivate() {
    cam_info_.reset();
    target_data_pub_.reset();
    image_sub_.reset();
    result_img_pub_.shutdown();
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

void HeliosAutoAim::init_markers() {
    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    if (params_.armor_autoaim) {
        // the number of armor which is detected in camera
        text_marker_.ns = "classification";
        text_marker_.action = visualization_msgs::msg::Marker::ADD;
        text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker_.scale.z = 0.1;
        text_marker_.color.a = 1.0;
        text_marker_.color.r = 1.0;
        text_marker_.color.g = 1.0;
        text_marker_.color.b = 1.0;
        text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
        // position of the center of target car
        position_marker_.ns = "position";
        position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
        position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
        position_marker_.color.a = 1.0;
        position_marker_.color.g = 1.0;
        // linear velocity of target car
        linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
        linear_v_marker_.ns = "linear_v";
        linear_v_marker_.scale.x = 0.03;
        linear_v_marker_.scale.y = 0.05;
        linear_v_marker_.color.a = 1.0;
        linear_v_marker_.color.r = 1.0;
        linear_v_marker_.color.g = 1.0;
        // angular velocity of target car
        angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
        angular_v_marker_.ns = "angular_v";
        angular_v_marker_.scale.x = 0.03;
        angular_v_marker_.scale.y = 0.05;
        angular_v_marker_.color.a = 1.0;
        angular_v_marker_.color.b = 1.0;
        angular_v_marker_.color.g = 1.0;
        // four armors of target car
        armor_marker_.ns = "armors";
        armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker_.scale.x = 0.03;
        armor_marker_.scale.z = 0.125;
        armor_marker_.color.a = 1.0;
        armor_marker_.color.r = 1.0;
        // init marker_pub_
        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);
    } else {
        ///TODO: energy visiualization
    }
}

void HeliosAutoAim::publish_markers(helios_rs_interfaces::msg::Target target) {
    if (params_.armor_autoaim) {
        // init header
        position_marker_.header = target.header;
        linear_v_marker_.header = target.header;
        angular_v_marker_.header = target.header;
        armor_marker_.header = target.header;
        visualization_msgs::msg::MarkerArray marker_array;
        // caculate state and publish markers
        if (target.tracking) {
            // template names
            double yaw = target.yaw, r1 = target.radius_1, r2 = target.radius_2;
            double xc = target.position.x, yc = target.position.y, zc = target.position.z;
            double vxc = target.velocity.x, vyc = target.velocity.y, vzc = target.velocity.z, vyaw = target.v_yaw;
            double dz_ = target.dz;
            // write position of car center
            position_marker_.action = visualization_msgs::msg::Marker::ADD;
            position_marker_.pose.position.x = xc;
            position_marker_.pose.position.y = yc;
            position_marker_.pose.position.z = zc + dz_ / 2;
            // write linear velocity of car
            linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
            linear_v_marker_.points.clear();
            linear_v_marker_.points.emplace_back(position_marker_.pose.position);
            geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
            arrow_end.x += vxc;
            arrow_end.y += vyc;
            arrow_end.z += vzc;
            linear_v_marker_.points.emplace_back(arrow_end);
            // write angular velocity of car
            angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
            angular_v_marker_.points.clear();
            angular_v_marker_.points.emplace_back(position_marker_.pose.position);
            arrow_end = position_marker_.pose.position;
            arrow_end.z += vyaw / M_PI;
            angular_v_marker_.points.emplace_back(arrow_end);
            // write armor marker
            armor_marker_.action = visualization_msgs::msg::Marker::ADD;
            armor_marker_.scale.y = target.armor_type == "SMALL" ? 0.135 : 0.23;
            bool is_current_pair = true;
            size_t a_n = static_cast<int>(target.armors_num) + 2;
            geometry_msgs::msg::Point p_a;
            double r = 0;
            for (size_t i = 0; i < a_n; i++) {
                double tmp_yaw = yaw + i * (2 * M_PI / a_n);
                // Only 4 armors has 2 radius and height
                if (a_n == 4) {
                    r = is_current_pair ? r1 : r2;
                    p_a.z = zc + (is_current_pair ? 0 : dz_);
                    is_current_pair = !is_current_pair;
                } else {
                    r = r1;
                    p_a.z = zc;
                }
                p_a.x = xc - r * cos(tmp_yaw);
                p_a.y = yc - r * sin(tmp_yaw);
                armor_marker_.id = i;
                armor_marker_.pose.position = p_a;
                tf2::Quaternion q;
                // number 8 is outpost
                q.setRPY(0, target.armors_num == 8 ? -0.26 : 0.26, tmp_yaw);
                armor_marker_.pose.orientation = tf2::toMsg(q);
                marker_array.markers.emplace_back(armor_marker_);
            }
        }
    } else {

    }
}

void HeliosAutoAim::image_callback(sensor_msgs::msg::Image::SharedPtr msg) {
    // check if params are updated
    if (param_listener_->is_old(params_)) {
        param_listener_->refresh_dynamic_parameters();
        params_ = param_listener_->get_params();
        detector_->set_params(params_.detector);
        predictor_->set_params(params_.predictor);
    }
    // convert image
    auto img = cv_bridge::toCvShare(msg, "rgb8")->image;
    // detect armors
    auto armors = detector_->detect_targets(img);
    if (params_.debug) {
        detector_->draw_results(img);
        /// TODO: publish debug images
        result_img_pub_.publish(cv_bridge::CvImage(msg->header, "mono8", img).toImageMsg());
    }
    if (armors.armors.empty()) {
        RCLCPP_DEBUG(logger_, "No armor detecter");
        return ;
    }
    // coordinate transform
    for (auto & armor : armors.armors) {
        geometry_msgs::msg::PointStamped point;
        point.header = armors.header;
        point.point = armor.pose.position;
        try {
            auto transform = tf2_buffer_->lookupTransform(
                armors.header.frame_id, params_.predictor.target_frame, armors.header.stamp);
            tf2::doTransform(point, point, transform);
            armor.pose.position = point.point;
        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(logger_, "Failed to transform armor pose: %s", ex.what());
            return ;
        }
    }
    // prediction
    auto target = predictor_->predict_target(armors, this->now());
    // publish gimbal instructions
    target_data_pub_->publish(target);
    // publish visiualization markers
    if (params_.debug) {
        publish_markers(target);
    }
}

} // namespace helios_cv

// register node to component
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::HeliosAutoAim);