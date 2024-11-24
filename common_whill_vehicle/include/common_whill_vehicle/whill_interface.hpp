#ifndef WHILL_INTERFACE_HPP__
#define WHILL_INTERFACE_HPP__

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "tier4_vehicle_msgs/msg/battery_status.hpp"
#include "whill_msgs/msg/model_cr2_state.hpp"

class WhillInterface : public rclcpp::Node
{

public:
    using Control = autoware_control_msgs::msg::Control;
    WhillInterface();

private:
    // Publisher
    rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_pub_;

    // Subscription
    rclcpp::Subscription<whill_msgs::msg::ModelCr2State>::SharedPtr whill_sub_;
    void whill_callback(const whill_msgs::msg::ModelCr2State::SharedPtr msg);

    // Client

    // Server

    // Param
    double wheel_base_;
    double wheel_tread_;
    double motor_speed_factor_;
    double min_velocity_threshold_;
    double max_velocity_threshold_;
    double min_steering_threshold_;
    double max_steering_threshold_;

    // function
    void get_parameters();

    // variable
};

#endif