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
#pragma once

#include <autoaim_interfaces/msg/detail/receive_data__struct.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <angles/angles.h>
#include <image_transport/image_transport.hpp>

#include <rclcpp/subscription_base.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <message_filters/subscriber.h>

#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <autoaim_interfaces/msg/armors.hpp>
#include <autoaim_interfaces/msg/target.hpp>
#include <autoaim_interfaces/msg/receive_data.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/quaternion.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#define BULLET_SPEED 27.0
#define BULLET_RADIUS 0.015
#define BULLET_INTERATE_NUM 5

#define AIR_COEFF 0.038

namespace helios_cv {

class AutoAimDebugger : public rclcpp::Node {
public:
    AutoAimDebugger(const rclcpp::NodeOptions& options);

    ~AutoAimDebugger() = default;

private:
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);

    void publish_detector_markers();

    void publish_target_markers();

    void bullistic_model();

    void draw_target();

    void init_markers();

    // publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<autoaim_interfaces::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Subscription<autoaim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<autoaim_interfaces::msg::ReceiveData>::SharedPtr receive_data_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detect_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr target_marker_pub_;

    image_transport::Publisher image_pub_;

    // tf2 utilities
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    geometry_msgs::msg::TransformStamped transform_stamped_;
    geometry_msgs::msg::TransformStamped yaw_pitch_ts_;
    
    // visualization markers
    visualization_msgs::msg::Marker detect_armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray detect_marker_array_;

    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker linear_v_marker_;
    visualization_msgs::msg::Marker angular_v_marker_;
    visualization_msgs::msg::Marker target_armor_marker_;
    visualization_msgs::msg::MarkerArray target_marker_array_;

    // armors and targets
    autoaim_interfaces::msg::Armors::SharedPtr armors_msg_;
    autoaim_interfaces::msg::Target::SharedPtr target_msg_;
    autoaim_interfaces::msg::ReceiveData::SharedPtr receive_data_msg_;

    // camera info
    cv::Point image_center_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_; 

    // project infos
    std::vector<cv::Point3f> object_points_;
    std::vector<cv::Point3f> bullet_object_points_;

    std::vector<cv::Mat> detect_tvecs_;
    std::vector<cv::Quatd> detect_rvecs_;
    std::vector<std::string> armor_text_;

    std::vector<geometry_msgs::msg::Pose> target_pose_ros_;
    std::vector<cv::Mat> target_tvecs_;
    std::vector<cv::Quatd> target_rvecs_;

    double target_distance_;
    std::vector<cv::Mat> bullet_tvecs_;
    std::vector<double> bullet_distance_;

    // raw image
    cv::Mat raw_image_;
};


} // namespace helios_cv