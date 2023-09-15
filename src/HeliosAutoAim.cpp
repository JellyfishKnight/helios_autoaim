#include "HeliosAutoAim.hpp"
#include <functional>
#include <helios_rs_interfaces/msg/detail/send_data__struct.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

namespace helios_cv {

HeliosAutoAim::HeliosAutoAim(const rclcpp::NodeOptions & options) : 
    LifecycleNode("helios_autoaim", options) {}

HeliosAutoAim::CallbackReturn HeliosAutoAim::on_configure(const rclcpp_lifecycle::State & state) {
    // Create publishers and subscribers
    target_data_pub_ = this->create_publisher<helios_rs_interfaces::msg::SendData>(
        "autoaim_cmd", rclcpp::SensorDataQoS());
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(), std::bind(&HeliosAutoAim::image_callback, this, std::placeholders::_1));
    // update params
    param_listener_ = std::make_shared<helios_autoaim::ParamListener>(this);
    params_ = param_listener_->get_params();
    // create detector
    
    return CallbackReturn::SUCCESS;
}

HeliosAutoAim::CallbackReturn HeliosAutoAim::on_activate(const rclcpp_lifecycle::State & state) {
    
    return CallbackReturn::SUCCESS;
}

HeliosAutoAim::CallbackReturn HeliosAutoAim::on_deactivate(const rclcpp_lifecycle::State & state) {

    return CallbackReturn::SUCCESS;
}

HeliosAutoAim::CallbackReturn HeliosAutoAim::on_cleanup(const rclcpp_lifecycle::State & state) {

    return CallbackReturn::SUCCESS;
}

HeliosAutoAim::CallbackReturn HeliosAutoAim::on_shutdown(const rclcpp_lifecycle::State & state) {

    return CallbackReturn::SUCCESS;
}

HeliosAutoAim::CallbackReturn HeliosAutoAim::on_error(const rclcpp_lifecycle::State & state) {

    return CallbackReturn::SUCCESS;
}

} // namespace helios_cv