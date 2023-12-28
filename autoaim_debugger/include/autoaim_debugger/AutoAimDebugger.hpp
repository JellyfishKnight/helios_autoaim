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

#include <autoaim_interfaces/msg/detail/armor__struct.hpp>
#include <autoaim_interfaces/msg/detail/armors__struct.hpp>
#include <autoaim_interfaces/msg/detail/target__struct.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <angles/angles.h>
#include <image_transport/image_transport.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <message_filters/subscriber.h>

#include <rclcpp/subscription_base.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <autoaim_interfaces/msg/armors.hpp>
#include <autoaim_interfaces/msg/target.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace helios_cv {

using tf2_filter = tf2_ros::MessageFilter<sensor_msgs::msg::Image>;

class AutoAimDebugger : public rclcpp::Node {
public:
    AutoAimDebugger(const rclcpp::NodeOptions& options);

    ~AutoAimDebugger() = default;

private:
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);

    void publish_detector_markers();

    void publish_target_markers();

    void draw_target();

    void init_markers();

    // publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<autoaim_interfaces::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Subscription<autoaim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detect_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr target_marker_pub_;

    image_transport::Publisher image_pub_;

    // tf2 utlities
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    std::shared_ptr<tf2_filter> tf2_filter_;


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

    // camera info
    cv::Point image_center_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_; 

    // raw image
    cv::Mat raw_image_;
};


} // namespace helios_cv