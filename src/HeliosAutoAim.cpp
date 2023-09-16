#include "HeliosAutoAim.hpp"
#include <functional>
#include <helios_rs_interfaces/msg/detail/send_data__struct.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

namespace helios_cv {

HeliosAutoAim::HeliosAutoAim(const rclcpp::NodeOptions& options) : 
    Node("helios_autoaim", options) {
    // update params
    param_listener_ = std::make_shared<helios_autoaim::ParamListener>(this);
    params_ = param_listener_->get_params();
    state_ = State::UNCONFIGURED;
    // state machine
    while (rclcpp::ok()) {
        if (params_.trasition == Trasition::CONFIGURE && state_ == State::UNCONFIGURED) {
            on_configure();
        }
    }
}

CallbackReturn HeliosAutoAim::on_configure() {
    // Create publishers and subscribers
    target_data_pub_ = this->create_publisher<helios_rs_interfaces::msg::SendData>(
        "autoaim_cmd", rclcpp::SensorDataQoS());
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(), std::bind(&HeliosAutoAim::image_callback, this, std::placeholders::_1));
    // create detector
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn HeliosAutoAim::on_activate() {
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn HeliosAutoAim::on_deactivate() {

    return CallbackReturn::SUCCESS;
}

CallbackReturn HeliosAutoAim::on_cleanup() {

    return CallbackReturn::SUCCESS;
}

CallbackReturn HeliosAutoAim::on_shutdown() {

    return CallbackReturn::SUCCESS;
}

CallbackReturn HeliosAutoAim::on_error() {
    
    return CallbackReturn::SUCCESS;
}

} // namespace helios_cv