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
#include <cmath>
#include <cstddef>
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <vector>


namespace helios_cv {
AutoAimDebugger::AutoAimDebugger(const rclcpp::NodeOptions& options) : rclcpp::Node("autoaim_debugger", options) {
    /// init markers
    init_markers();
    /// create publishers
    detect_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/autoaim_debugger/detect_markers", 10);
    target_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/autoaim_debugger/target_markers", 10);
    image_pub_ = image_transport::create_publisher(this, "/autoaim_debugger/result_img");
    /// create tf2 utilities
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    /// create subscribers
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
            camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
            distortion_coefficients_ = cv::Mat(1, 5, CV_64FC1, const_cast<double*>(msg->d.data())).clone();
            image_center_ = cv::Point2d(camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
            camera_info_sub_.reset();
        }
    );
    armors_sub_ = this->create_subscription<autoaim_interfaces::msg::Armors>("/detector/armors", 10,
        [this](autoaim_interfaces::msg::Armors::SharedPtr msg) {
            armors_msg_ = std::move(msg);
        }
    );
    target_sub_ = this->create_subscription<autoaim_interfaces::msg::Target>("/predictor/target", 10,
        [this](autoaim_interfaces::msg::Target::SharedPtr msg) {
            target_msg_ = std::move(msg);
        }
    );
    receive_data_sub_ = this->create_subscription<autoaim_interfaces::msg::ReceiveData>("/predictor/receive_data", 10,
        [this](autoaim_interfaces::msg::ReceiveData::SharedPtr msg) {
            receive_data_msg_ = std::move(msg);
        }
    );
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&AutoAimDebugger::image_callback, this, std::placeholders::_1)
    );
    RCLCPP_WARN(this->get_logger(), "Finished initializing");
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
    if (!(armors_msg_ == nullptr || target_msg_ == nullptr || target_msg_->tracking == false)) {
        /// transform target from odom to camera
        try {
            transform_stamped_ = tf2_buffer_->lookupTransform("camera_optical_frame", "odom", msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
            yaw_pitch_ts_ = tf2_buffer_->lookupTransform("gimbal_link", "odom", msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
        } catch (tf2::ExtrapolationException& e) {
            RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
            return;
        }
        /// Publish markers
        publish_detector_markers();
        publish_target_markers();
        /// Caculate bullet tracks
        bullistic_model();
        /// Draw debug infos on image
        draw_target();
    }
    image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, raw_image_).toImageMsg());
}


void AutoAimDebugger::publish_detector_markers() {
    /// Clear array
    detect_marker_array_.markers.clear();
    detect_rvecs_.clear();
    detect_tvecs_.clear();
    armor_text_.clear();
    /// Push detect armor markers
    detect_armor_marker_.id = text_marker_.id = 0;
    if (!armors_msg_->armors.empty()) {
        for (auto armor : armors_msg_->armors) {
            detect_armor_marker_.header = armors_msg_->header;
            detect_armor_marker_.pose = armor.pose;
            detect_armor_marker_.id++;
            detect_armor_marker_.scale.y = armor.type == 0 ? 0.135 : 0.23;
            detect_marker_array_.markers.push_back(detect_armor_marker_);
            text_marker_.header = armors_msg_->header;
            text_marker_.pose = armor.pose;
            text_marker_.pose.position.z += 0.1;
            text_marker_.id++;
            text_marker_.text = armor.number;
            detect_marker_array_.markers.push_back(text_marker_);

            armor_text_.emplace_back(armor.number);
            cv::Mat tvec = (cv::Mat_<double>(3, 1) << armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
            detect_tvecs_.emplace_back(tvec);
            cv::Quatd q(armor.pose.orientation.w, armor.pose.orientation.x, armor.pose.orientation.y, armor.pose.orientation.z);
            detect_rvecs_.emplace_back(q);
        }
    } else {
        detect_armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
        text_marker_.action = visualization_msgs::msg::Marker::DELETE;
    }
    detect_armor_marker_.action = armors_msg_->armors.empty() ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
    detect_marker_array_.markers.emplace_back(detect_armor_marker_);
    detect_marker_pub_->publish(detect_marker_array_);
}

void AutoAimDebugger::publish_target_markers() {
    /// Clear marker array
    target_marker_array_.markers.clear();
    target_pose_ros_.clear();
    target_rvecs_.clear();
    target_tvecs_.clear();
    /// Push target marker
    target_armor_marker_.header = target_msg_->header;
    position_marker_.header = target_msg_->header;
    linear_v_marker_.header = target_msg_->header;
    angular_v_marker_.header = target_msg_->header;
    if (target_msg_->tracking) {
        double yaw = target_msg_->yaw, r1 = target_msg_->radius_1, r2 = target_msg_->radius_2;
        double xc = target_msg_->position.x, yc = target_msg_->position.y, zc = target_msg_->position.z;
        double vxc = target_msg_->velocity.x, vyc = target_msg_->velocity.y, vzc = target_msg_->velocity.z, vyaw = target_msg_->v_yaw;
        double dz = target_msg_->dz;

        target_distance_ = std::sqrt(xc * xc + yc * yc);
        // RCLCPP_WARN(this->get_logger(), "xc %f yc %f zc %f", xc, yc, zc);

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

            // Get target armor corners
            target_pose_ros_.emplace_back(target_armor_marker_.pose);
        }
    } else {
        position_marker_.action = visualization_msgs::msg::Marker::DELETE;
        linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
        angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
        target_armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    }
    target_marker_array_.markers.emplace_back(position_marker_);
    target_marker_array_.markers.emplace_back(linear_v_marker_);
    target_marker_array_.markers.emplace_back(angular_v_marker_);

    target_marker_pub_->publish(target_marker_array_);
}

void AutoAimDebugger::draw_target() {
    /// Draw detect armors
    for (std::size_t i = 0; i < detect_tvecs_.size(); i++) {
        std::vector<cv::Point2f> image_points;
        cv::projectPoints(object_points_, detect_rvecs_[i].toRotVec(), detect_tvecs_[i], camera_matrix_, distortion_coefficients_, image_points);
        for (size_t i = 0; i < image_points.size(); i++) {
            cv::line(raw_image_, image_points[i], image_points[(i + 1) % image_points.size()], cv::Scalar(0, 255, 0), cv::LINE_4);
        }
        cv::line(raw_image_, image_points[0], image_points[2], cv::Scalar(0, 255, 0), cv::LINE_4);
        cv::line(raw_image_, image_points[1], image_points[3], cv::Scalar(0, 255, 0), cv::LINE_4);
        cv::putText(raw_image_, armor_text_[i], image_points[1], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
    }

    /// Transform targets from odom to camera
    try {
        for (auto& pose : target_pose_ros_) {    
            tf2::doTransform(pose, pose, transform_stamped_);
        }
    } catch (tf2::TransformException& e) {
        RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
        return;
    }
    /// Draw target armors
    // convert ros pose to cv pose
    for (auto& pose : target_pose_ros_) {
        cv::Mat tvec = (cv::Mat_<double>(3, 1) << pose.position.x, pose.position.y, pose.position.z);
        target_tvecs_.emplace_back(tvec);
        cv::Quatd q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        target_rvecs_.emplace_back(q);
    }
    for (std::size_t i = 0; i < target_tvecs_.size(); i++) {
        std::vector<cv::Point2f> image_points;
        cv::projectPoints(object_points_, target_rvecs_[i].toRotMat3x3(), target_tvecs_[i], camera_matrix_, distortion_coefficients_, image_points);
        cv::putText(raw_image_, std::to_string(i), image_points[2], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        for (size_t i = 0; i < image_points.size(); i++) {
            cv::line(raw_image_, image_points[i], image_points[(i + 1) % image_points.size()], cv::Scalar(0, 0, 255), cv::LINE_4);
        }
    }
    /// Transform bullets from odom to camera
    std::vector<cv::Quatd> rvecs;
    try {
        for (auto tvec : bullet_tvecs_) {    
            geometry_msgs::msg::Pose point;
            point.orientation.w = 1;
            point.orientation.x = 0;
            point.orientation.y = 0;
            point.orientation.z = 0;
            point.position.x = tvec.at<double>(0, 0);
            point.position.y = tvec.at<double>(1, 0);
            point.position.z = tvec.at<double>(2, 0);
            tf2::doTransform(point, point, transform_stamped_);
            tvec.at<double>(0, 0) = point.position.x;
            tvec.at<double>(1, 0) = point.position.y;
            tvec.at<double>(2, 0) = point.position.z;
            rvecs.emplace_back(cv::Quatd{point.orientation.w, point.orientation.x, point.orientation.y, point.orientation.z});
        }
    } catch (tf2::TransformException& e) {
        RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
        return;
    }
    /// Draw Bullet tracks
    for (std::size_t i = 1; i < bullet_tvecs_.size(); i++) {
        std::vector<cv::Point2f> image_points;
        cv::projectPoints(bullet_object_points_, rvecs[i].toRotMat3x3(), bullet_tvecs_[i], camera_matrix_, distortion_coefficients_, image_points);
        double radius = cv::norm(image_points[0] - image_points[1]) / std::sqrt(2);
        cv::Point2f bullet_center = (image_points[0] + image_points[1] + image_points[2] + image_points[3]) / 4;
        try {
            cv::circle(raw_image_, bullet_center, radius, cv::Scalar(255, 0, 0), 4);
            cv::putText(raw_image_, std::to_string(bullet_distance_[i]), image_points[2], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        } catch (cv::Exception& e) {
            // RCLCPP_INFO(this->get_logger(), "center x %f y %f", bullet_center.x, bullet_center.y);
        }
    }
    /// Draw Prediction Point
    if (receive_data_msg_ != nullptr) {
        geometry_msgs::msg::Point point;
        point.x = receive_data_msg_->x;
        point.y = receive_data_msg_->y;
        point.z = receive_data_msg_->z;
        try {
            tf2::doTransform(point, point, transform_stamped_);
        } catch (tf2::TransformException& e) {
            RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
            return;
        }
        cv::Mat_<double> predict_point = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
        predict_point = (camera_matrix_ / predict_point.at<double>(2)) * predict_point;
        cv::circle(raw_image_, cv::Point2f(predict_point.at<double>(0), predict_point.at<double>(1)), 5, cv::Scalar(255, 0, 0), 2);
    }
    /// Draw image center
    cv::circle(raw_image_, image_center_, 5, cv::Scalar(0, 0, 255), 2);
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

    double armor_half_y = 135 / 2.0 / 1000.0;
    double armor_half_z = 125 / 2.0 / 1000.0;

    object_points_.emplace_back(cv::Point3f(0, armor_half_y, -armor_half_z));
    object_points_.emplace_back(cv::Point3f(0, armor_half_y, armor_half_z));
    object_points_.emplace_back(cv::Point3f(0, -armor_half_y, armor_half_z));
    object_points_.emplace_back(cv::Point3f(0, -armor_half_y, -armor_half_z));

    bullet_object_points_.emplace_back(cv::Point3f(0, BULLET_RADIUS, 0));
    bullet_object_points_.emplace_back(cv::Point3f(0, -BULLET_RADIUS, 0));
    bullet_object_points_.emplace_back(cv::Point3f(0, 0, BULLET_RADIUS));
    bullet_object_points_.emplace_back(cv::Point3f(0, 0, -BULLET_RADIUS));
}

void AutoAimDebugger::bullistic_model() {
    bullet_tvecs_.clear();
    bullet_distance_.clear();
    // get pitch
    double x, y, z, w;
    x = yaw_pitch_ts_.transform.rotation.x;
    y = yaw_pitch_ts_.transform.rotation.y;
    z = yaw_pitch_ts_.transform.rotation.z;
    w = yaw_pitch_ts_.transform.rotation.w;
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // caculate bullet positions
    double vel_x = BULLET_SPEED * std::cos(pitch);
    double vel_z = BULLET_SPEED * std::sin(pitch);
    double distance_slice = target_distance_ / BULLET_INTERATE_NUM;
    // RCLCPP_WARN(this->get_logger(), "vx %f vz %f", vel_x, vel_z);
    for (double i = 0; i <= target_distance_; i += distance_slice) {
        double time = (exp(AIR_COEFF * i) - 1) / vel_x / AIR_COEFF;
        double temp_x = i * std::cos(-yaw);
        double temp_y = i * std::sin(-yaw);
        double temp_z = vel_z * time - 0.5 * 9.8 * time * time;
        // if (i == target_distance_)
        //     RCLCPP_INFO(this->get_logger(), "bullet x %f y %f z %f", temp_x, temp_y, temp_z); // use the output only when you need to debug
        cv::Mat p(3, 1, CV_64FC1);
        p.at<double>(0, 0) = temp_x;
        p.at<double>(1, 0) = temp_y;
        p.at<double>(2, 0) = temp_z;
        bullet_tvecs_.emplace_back(p);
        bullet_distance_.emplace_back(i);
    }
}


} // namespace helios_cv

// register node to component
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::AutoAimDebugger);