// created by liuhan on 2023/12/28
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#include "AutoAimDebugger.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <tf2/exceptions.h>


namespace helios_cv {
AutoAimDebugger::AutoAimDebugger(const rclcpp::NodeOptions& options) : rclcpp::Node("autoaim_debugger", options) {
    /// init markers
    init_markers();
    /// create publishers
    detect_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/autoaim_debugger/detect_markers", 10);
    target_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/autoaim_debugger/target_markers", 10);
    image_pub_ = image_transport::create_publisher(this, "/autoaim_debugger/image");
    /// create subscribers
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
            camera_matrix_ = cv::Mat(3, 3, CV_64FC1, const_cast<double*>(msg->k.data()));
            distortion_coefficients_ = cv::Mat(1, 5, CV_64FC1, const_cast<double*>(msg->d.data()));
            image_center_ = cv::Point2d(camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
            camera_info_sub_.reset();
        }
    );
    armors_sub_ = this->create_subscription<autoaim_interfaces::msg::Armors>("armors", 10,
        [this](autoaim_interfaces::msg::Armors::SharedPtr msg) {
            armors_msg_ = std::move(msg);
            publish_detector_markers();
        }
    );
    target_sub_ = this->create_subscription<autoaim_interfaces::msg::Target>("target", 10,
        [this](autoaim_interfaces::msg::Target::SharedPtr msg) {
            target_msg_ = std::move(msg);
            publish_target_markers();
        }
    );
    // init tf2 utilities
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // subscriber and filter    
    image_sub_.subscribe(this, "/image_raw", rmw_qos_profile_sensor_data);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_ = std::make_shared<tf2_filter>(
        image_sub_, *tf2_buffer_, "camera_optical_frame", 10, this->get_node_logging_interface(),
        this->get_node_clock_interface(), std::chrono::duration<int>(2));
}

void AutoAimDebugger::image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    /// Convert image
    try {
        raw_image_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    /// Check camera info
    if (camera_matrix_.empty() || distortion_coefficients_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "camera matrix or distortion coefficients is empty");
        return;
    }
    /// Check if armors or target is empty
    if (armors_msg_ == nullptr || target_msg_ == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "armors or target is empty");
        return;
    }
    /// transform target from odom to camera
    try {

    } catch (tf2::ExtrapolationException& e) {
        RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
        return;
    }
    draw_target();
    image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, raw_image_).toImageMsg());
}


void AutoAimDebugger::publish_detector_markers() {
    /// Clear marker array
    detect_marker_array_.markers.clear();
    /// Push detect armor markers
    detect_armor_marker_.id = text_marker_.id = 0;
    if (!armors_msg_->armors.empty()) {
        for (auto armor : armors_msg_->armors) {
            detect_armor_marker_.header = armors_msg_->header;
            detect_armor_marker_.pose.position.x = armor.pose.position.x;
            detect_armor_marker_.pose.position.y = armor.pose.position.y;
            detect_armor_marker_.pose.position.z = armor.pose.position.z;
            detect_armor_marker_.pose.orientation.x = armor.pose.orientation.x;
            detect_armor_marker_.pose.orientation.y = armor.pose.orientation.y;
            detect_armor_marker_.pose.orientation.z = armor.pose.orientation.z;
            detect_armor_marker_.pose.orientation.w = armor.pose.orientation.w;
            detect_armor_marker_.id++;
            detect_marker_array_.markers.push_back(detect_armor_marker_);
            text_marker_.header = armors_msg_->header;
            text_marker_.pose.position.x = armor.pose.position.x;
            text_marker_.pose.position.y = armor.pose.position.y;
            text_marker_.pose.position.z = armor.pose.position.z + 0.1;
            text_marker_.pose.orientation.x = armor.pose.orientation.x;
            text_marker_.pose.orientation.y = armor.pose.orientation.y;
            text_marker_.pose.orientation.z = armor.pose.orientation.z;
            text_marker_.pose.orientation.w = armor.pose.orientation.w;
            text_marker_.id++;
            text_marker_.text = armor.number;
            detect_marker_array_.markers.push_back(text_marker_);
        }
    } else {
        detect_armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
        text_marker_.action = visualization_msgs::msg::Marker::DELETE;
        detect_marker_array_.markers.push_back(detect_armor_marker_);
        detect_marker_array_.markers.push_back(text_marker_);
    }
    detect_marker_pub_->publish(detect_marker_array_);
}

void AutoAimDebugger::publish_target_markers() {
    /// Clear marker array
    target_marker_array_.markers.clear();
    /// Push target marker
    target_armor_marker_.header = target_msg_->header;
    if (target_msg_->tracking) {
        double yaw = target_msg_->yaw, r1 = target_msg_->radius_1, r2 = target_msg_->radius_2;
        double xc = target_msg_->position.x, yc = target_msg_->position.y, zc = target_msg_->position.z;
        double vxc = target_msg_->velocity.x, vyc = target_msg_->velocity.y, vzc = target_msg_->velocity.z, vyaw = target_msg_->v_yaw;
        double dz = target_msg_->dz;

        position_marker_.action = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position.x = xc;
        position_marker_.pose.position.y = yc;
        position_marker_.pose.position.z = zc + dz / 2;

        linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += vxc;
        arrow_end.y += vyc;
        arrow_end.z += vzc;
        linear_v_marker_.points.emplace_back(arrow_end);

        angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += vyaw / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);

        target_armor_marker_.action = visualization_msgs::msg::Marker::ADD;
        target_armor_marker_.scale.y = target_msg_->armor_type == "SMALL" ? 0.135 : 0.23;
        bool is_current_pair = true;
        size_t a_n = target_msg_->armors_num;
        geometry_msgs::msg::Point p_a;
        double r = 0;
        for (size_t i = 0; i < a_n; i++) {
            double tmp_yaw = yaw + i * (2 * M_PI / a_n);
            // Only 4 armors has 2 radius and height
            if (a_n == 4) {
                r = is_current_pair ? r1 : r2;
                p_a.z = zc + (is_current_pair ? 0 : dz);
                is_current_pair = !is_current_pair;
            } else {
                r = r1;
                p_a.z = zc;
            }
            p_a.x = xc - r * cos(tmp_yaw);
            p_a.y = yc - r * sin(tmp_yaw);

            target_armor_marker_.id = i;
            target_armor_marker_.pose.position = p_a;
            tf2::Quaternion q;
            q.setRPY(0, target_msg_->id == "outpost" ? -0.26 : 0.26, tmp_yaw);
            target_armor_marker_.pose.orientation = tf2::toMsg(q);
            target_marker_array_.markers.emplace_back(target_armor_marker_);
        }
    } else {
        position_marker_.action = visualization_msgs::msg::Marker::DELETE;
        linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
        angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
        target_armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    }
    target_marker_pub_->publish(target_marker_array_);
}

void AutoAimDebugger::draw_target() {
    /// Draw image center
    cv::circle(raw_image_, image_center_, 5, cv::Scalar(0, 0, 255), 2);
    /// Draw detected armors
    for (auto& armor : armors_msg_->armors) {
        cv::Point2f vertices[4];
        for (int i = 0; i < 4; i++) {
            vertices[i].x = armor.points[i].x;
            vertices[i].y = armor.points[i].y;
        }
        for (int i = 0; i < 4; i++) {
            cv::line(raw_image_, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
        cv::putText(raw_image_, armor.number, vertices[0], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    }
    /// Draw target
    
}

void AutoAimDebugger::init_markers() {
    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    detect_armor_marker_.ns = "armors";
    detect_armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    detect_armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    detect_armor_marker_.scale.x = 0.05;
    detect_armor_marker_.scale.z = 0.125;
    detect_armor_marker_.color.a = 1.0;
    detect_armor_marker_.color.g = 0.5;
    detect_armor_marker_.color.b = 1.0;
    detect_armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    text_marker_.ns = "classification";
    text_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z = 0.1;
    text_marker_.color.a = 1.0;
    text_marker_.color.r = 1.0;
    text_marker_.color.g = 1.0;
    text_marker_.color.b = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;
    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;
    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 1.0;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;
    target_armor_marker_.ns = "armors";
    target_armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    target_armor_marker_.scale.x = 0.03;
    target_armor_marker_.scale.z = 0.125;
    target_armor_marker_.color.a = 1.0;
    target_armor_marker_.color.r = 1.0;
}



} // namespace helios_cv

// register node to component
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::AutoAimDebugger);