// created by liuhan on 2023/11/4
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#include "FireControllerNode.hpp"


namespace helios_cv {

FireController::FireController(const rclcpp::NodeOptions& options) : 
    rclcpp::Node("fire_controller", options) {
    // create param
    try {
        param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
        params_ = param_listener_->get_params();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to start param listener: %s", e.what());
    }
    // create utilities
    bullet_solver_ = std::make_shared<BulletSolver>();
    target_solver_ = std::make_shared<TargetSolver>();
    if (!bullet_solver_ || !target_solver_) {
        RCLCPP_ERROR(logger_, "Failed to create utilities, try again");
        bullet_solver_ = std::make_shared<BulletSolver>();
        target_solver_ = std::make_shared<TargetSolver>();
    }
    bullet_solver_->update_bullet_params(BulletParams{
        params_.bullet_solver.bullet_speed,
        params_.bullet_solver.bullet_mass,
        params_.bullet_solver.air_coeff
    });
    // create publisher
    serial_pub_ = this->create_publisher<autoaim_interfaces::msg::SendData>("serial_send", 10);
    gimbal_pub_ = this->create_publisher<helios_control_interfaces::msg::GimbalCmd>("gimbal_cmd", 10);
    shoot_pub_ = this->create_publisher<helios_control_interfaces::msg::ShooterCmd>("shooter_cmd", 10);
    // create subscriber
    if (params_.under_helios_rs) {
        imu_sub_ = this->create_subscription<sensor_interfaces::msg::ImuEuler>(
            "imu_euler", 10, std::bind(&FireController::imu_euler_callback, this, std::placeholders::_1));
        // bullet_sub_ = this->create_subscription<referee_interfaces::msg::>(, , )
    } else {
        serial_sub_ = this->create_subscription<autoaim_interfaces::msg::ReceiveData>(
            "serial_data", rclcpp::SensorDataQoS(), std::bind(&FireController::serial_callback, this, std::placeholders::_1));
    }
    // // 初始化tf2相关
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // // Create the timer interface before call to waitForTransform,
    // // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // // subscriber and filter
    armors_sub_.subscribe(this, "/predictor/target", rmw_qos_profile_sensor_data);
    tf2_filter_ = std::make_shared<tf2_filter>(
        armors_sub_, *tf2_buffer_, params_.target_frame, 10, this->get_node_logging_interface(),
        this->get_node_clock_interface(), std::chrono::duration<int>(2));
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&FireController::target_callback, this);        
    // create process thread
    std::thread{
        [this]() {
            rclcpp::Rate rate(300);
            while (rclcpp::ok()) {
                target_process();
                rate.sleep();
            }
        }
    }.detach();
}

FireController::~FireController() {

}

void FireController::imu_euler_callback(sensor_interfaces::msg::ImuEuler::SharedPtr imu_msg) {
    // update
    ypr_ = Eigen::Vector3d{imu_msg->yaw, imu_msg->pitch, imu_msg->roll};
}

void FireController::target_callback(autoaim_interfaces::msg::Target::SharedPtr target_msg) { 
    target_msg_ = target_msg;
}

void FireController::serial_callback(autoaim_interfaces::msg::ReceiveData::SharedPtr serial_msg) {
    ypr_(0) = serial_msg->yaw_angle;
    ypr_(1) = serial_msg->pitch_angle;
    ypr_(2) = 0;
    bullet_solver_->update_bullet_speed(serial_msg->bullet_speed);
}

void FireController::target_process() {
    if (!target_msg_->tracking || !target_msg_) {
        // send data when under traditional mode
        if (!params_.under_helios_rs) {
            autoaim_interfaces::msg::SendData send_data;
            send_data.find = false;
            send_data.cmd = false;
            send_data.number = 0;
            serial_pub_->publish(send_data);
        }
    }
    // check if msg has expired
    if ((this->now() - target_msg_->header.stamp).seconds() > params_.message_expire_time) {
        if (!params_.under_helios_rs) {
            autoaim_interfaces::msg::SendData send_data;
            send_data.find = false;
            send_data.cmd = false;
            send_data.number = 0;
            serial_pub_->publish(send_data);
        }
        return;
    }
    // caculate new position
    Eigen::Vector3d predicted_xyz = target_solver_->get_best_armor(target_msg_, ypr_(0), latency_);
    // caculate fly time and pitch compensation
    double fly_time = 0;
    // caculate compute time cost
    double total_latency = (this->now() - target_msg_->header.stamp).seconds();
    if (params_.debug) {
        RCLCPP_INFO(logger_, "total latency: %f", total_latency);
    }
    if (params_.under_helios_rs) {
        // caculate yaw and pitch
        double pitch = bullet_solver_->iterate_pitch(predicted_xyz, fly_time);
        double yaw = std::atan2(predicted_xyz(1), predicted_xyz(0));
        // update gimbal cmd
        auto gimbal_cmd = std::make_unique<helios_control_interfaces::msg::GimbalCmd>();
        gimbal_cmd->yaw = yaw;
        gimbal_cmd->pitch = pitch;
        gimbal_pub_->publish(std::move(gimbal_cmd));
        // update shoot cmd
        auto shoot_cmd_msg = std::make_unique<helios_control_interfaces::msg::ShooterCmd>();
        shoot_cmd_msg->shooter_mode = 2;
        shoot_cmd_msg->dial_mode = 2;
        // judge shoot cmd
        bool shoot_cmd = judge_shoot_cmd(predicted_xyz.norm(), yaw, pitch); 
        shoot_cmd_msg->fire_flag = shoot_cmd ? 1 : 0;
        shoot_cmd_msg->dial_velocity_level = 10;
        shoot_pub_->publish(std::move(shoot_cmd_msg));
    } else {
        // transform to camera coordinate
        geometry_msgs::msg::PointStamped predicted_point;
        predicted_point.header.frame_id = params_.target_frame;
        predicted_xyz(0) = predicted_point.point.x;
        predicted_xyz(1) = predicted_point.point.y;
        predicted_xyz(2) = predicted_point.point.z;
        // pitch can be a absolute value
        double pitch = bullet_solver_->iterate_pitch(predicted_xyz, fly_time);
        // transform wanted point to camera
        try {
            predicted_point.point = tf2_buffer_->transform(predicted_point, params_.target_frame).point;
        } catch (tf2::ExtrapolationException &ex) {
            RCLCPP_ERROR(logger_, "Error while transforming %s", ex.what());
            return; 
        }
        predicted_point.point.x = predicted_xyz(0);
        predicted_point.point.y = predicted_xyz(1);
        predicted_point.point.z = predicted_xyz(2);
        // yaw should be total angle
        double yaw = std::atan2(predicted_xyz(1), predicted_xyz(0));
        // update serial send
        auto serial_send = std::make_unique<autoaim_interfaces::msg::SendData>();
        serial_send->yaw = yaw + ypr_(0);
        serial_send->pitch = pitch;
        // 2 is counter shoot mode
        // judge shoot cmd
        bool shoot_cmd = judge_shoot_cmd(predicted_xyz.norm(), yaw, pitch); 
        serial_send->cmd = shoot_cmd ? 2 : 0;
        serial_send->find = true;
        if (target_msg_->id[0] <= '9' && target_msg_->id[0] >= '0') {
            serial_send->number = target_msg_->id[0] - '0';
        } else if (target_msg_->id[0] == 'o') {
            serial_send->number = 8;
        } else if (target_msg_->id[0] == 'g') {
            serial_send->number = 7;
        } else if (target_msg_->id[0] == 'b') {
            serial_send->number = 9;
        }
        serial_pub_->publish(std::move(serial_send));
    }
    // update predict latency
    latency_ = params_.latency + fly_time + total_latency;
}

bool FireController::judge_shoot_cmd(double distance, double yaw, double pitch) {
    if (target_msg_->armors_num == OUTPOST_ARMOR_NUM && params_.is_hero) {
        ///TODO: 英雄前哨战开火判断

    } else {
        double armor_width = (target_msg_->armor_type == "LARGE" ? 0.23f : 0.135);
        double armor_height = 0.125f * 3;
        double yaw_error_threshold = std::atan2(armor_width / 2, distance) / (M_PI / 180.0);
        double pitch_error_threshold = std::atan2(armor_height / 2, distance) / (M_PI / 180.0);
        if (std::abs(yaw) < yaw_error_threshold &&
            std::abs(pitch) < pitch_error_threshold) {
            return true;
        } else {
            return false;    
        }
    }
}

} // namespace helios_cv