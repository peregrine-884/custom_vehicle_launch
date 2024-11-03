#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control.hpp"

using Control = autoware_control_msgs::msg::Control;

class ControlSubscriber : public rclcpp::Node
{
public:
  ControlSubscriber() : Node("control_subscriber")
  {
    // Subscription to /control/command/control_cmd topic
    autoware_sub_ = this->create_subscription<Control>(
      "/control/command/control_cmd", 1,
      std::bind(&ControlSubscriber::autoware_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Control Subscriber Node has been started.");
  }

private:
  void autoware_callback(const Control::SharedPtr msg)
  {
    // Display Lateral control data
    RCLCPP_INFO(this->get_logger(), "Received Lateral Control Command:");
    RCLCPP_INFO(this->get_logger(), "  Steering Tire Angle: %f", msg->lateral.steering_tire_angle);
    RCLCPP_INFO(this->get_logger(), "  Steering Tire Rotation Rate: %f", msg->lateral.steering_tire_rotation_rate);
    RCLCPP_INFO(this->get_logger(), "  Is Steering Rotation Rate Defined: %s", msg->lateral.is_defined_steering_tire_rotation_rate ? "true" : "false");

    // Display Longitudinal control data
    RCLCPP_INFO(this->get_logger(), "Received Longitudinal Control Command:");
    RCLCPP_INFO(this->get_logger(), "  Velocity: %f", msg->longitudinal.velocity);
    RCLCPP_INFO(this->get_logger(), "  Acceleration: %f", msg->longitudinal.acceleration);
    RCLCPP_INFO(this->get_logger(), "  Jerk: %f", msg->longitudinal.jerk);
    RCLCPP_INFO(this->get_logger(), "  Is Acceleration Defined: %s", msg->longitudinal.is_defined_acceleration ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Is Jerk Defined: %s", msg->longitudinal.is_defined_jerk ? "true" : "false");
  }

  rclcpp::Subscription<Control>::SharedPtr autoware_sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlSubscriber>());
  rclcpp::shutdown();
  return 0;
}
