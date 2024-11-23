#ifndef WHILL_CONTROLLER_HPP__
#define WHILL_CONTROLLER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"

class WhillController : public rclcpp::Node
{

public:
  using Control = autoware_control_msgs::msg::Control;
  using ActuationCommandStamped = tier4_vehicle_msgs::msg::ActuationCommandStamped;
  using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;

  WhillController();

private:
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr control_mode_pub_;

  // Subscription
  rclcpp::Subscription<Control>::SharedPtr autoware_sub_;
  void autoware_callback(const Control::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  void joy_callback(const sensor_msgs::msg::Joy & msg);

  // Client

  // Server

  // Param
  void get_parameters();
  double min_speed_, max_speed_;
  double min_angle_, max_angle_;
  double normalizer_;

  int manual_button_;
  int speed_plus_button_, speed_minus_button_;
  int speed_axes_, angle_axes_;

  // function
  void timer_callback();
  void publish_control_mode(uint8_t mode);

  // variable
  rclcpp::TimerBase::SharedPtr timer_;
  double speed_limit_factor_;
  bool plus_, minus_;
  bool manual_flag_;
};

#endif