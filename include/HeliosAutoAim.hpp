// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

#include <helios_rs_interfaces/msg/detail/target__struct.hpp>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
// tf2 
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <opencv2/core/mat.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>
// image
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
// interfaces
#include "helios_rs_interfaces/msg/target.hpp"
#include "helios_rs_interfaces/msg/receive_data.hpp"
#include "helios_rs_interfaces/msg/debug_lights.hpp"
#include "helios_rs_interfaces/msg/debug_armors.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
// opencv
#include <opencv2/core.hpp>
// functional modules
#include "TraditionalArmorDetector.hpp"
#include "TraditionalEnergyDetector.hpp"
#include "ArmorPredictor.hpp"
#include "EnergyPredictor.hpp"
// auto generated by ros2 generate_parameter_library
// https://github.com/PickNikRobotics/generate_parameter_library
#include "helios_autoaim_parameters.hpp"


namespace helios_cv {

enum Transition {
    NONE,
    CONFIGURE,
    ACTIVATE,
    DEACTIVATE,
    CLEANUP,
    SHUTDOWN,
};

enum State {
    UNCONFIGURED,
    INACTIVE,
    ACTIVE,
    FINALIZED,
    ERROR,
};

class HeliosAutoAim : public rclcpp::Node {
public:
    HeliosAutoAim(const rclcpp::NodeOptions& options);
private:
    // state utilities
    // set private to avoid directly editing transition and state
    Transition transition_;
    State state_;
public:
    State on_configure();

    State on_activate();

    State on_deactivate();

    State on_cleanup();

    State on_shutdown();

    State on_error();
private:
    // main functional modules
    std::shared_ptr<BaseDetector> detector_;
    std::shared_ptr<BasePredictor> predictor_; 
    // image about    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    /**
     * @brief image call back, main task function of this node
     * 
     * @param msg 
     */
    void image_callback(sensor_msgs::msg::Image::SharedPtr msg);

    // Visualization marker publisher
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;
    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker linear_v_marker_;
    visualization_msgs::msg::Marker angular_v_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    void init_markers();
    void publish_markers(helios_rs_interfaces::msg::Target target);

    // Cam Info
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;

    // tf utilities
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    
    // Debug information
    std::shared_ptr<image_transport::Publisher> binary_img_pub_;
    std::shared_ptr<image_transport::Publisher> number_img_pub_;
    std::shared_ptr<image_transport::Publisher> result_img_pub_;
    // Param listener
    rclcpp::Node* this_node_;
    std::shared_ptr<helios_autoaim::ParamListener> param_listener_;
    helios_autoaim::Params params_;

    // actually instruction to be send
    rclcpp::Publisher<helios_rs_interfaces::msg::Target>::SharedPtr target_data_pub_;
    
    // logger
    rclcpp::Logger logger_ = rclcpp::get_logger("helios_autoaim");
};

} // namespace helios_cv

