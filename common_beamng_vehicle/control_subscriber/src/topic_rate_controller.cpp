#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control.hpp"

using Control = autoware_control_msgs::msg::Control;

class TopicRateController : public rclcpp::Node
{
public:
  TopicRateController() : Node("topic_rate_controller")
  {
    // Subscription to /control/command/control_cmd topic
    control_cmd_sub_ = this->create_subscription<Control>(
      "/control/command/control_cmd", 1,
      std::bind(&TopicRateController::control_cmd_callback, this, std::placeholders::_1)
    );

    // Publisher to /rate_limitted/control/command/control_cmd topic
    control_cmd_pub_ = this->create_publisher<Control>(
      "/rate_limitted/control/command/control_cmd", 1
    );

    // parameter
    this->declare_parameter("control_cmd_rate", 10);
    control_cmd_rate_ = this->get_parameter("control_cmd_rate").as_int();

    // Timer to publish control command at a fixed rate
    control_cmd_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / control_cmd_rate_),
      std::bind(&TopicRateController::control_cmd_timer_callback, this)
    );
  }

private:
  void control_cmd_callback(const Control::SharedPtr msg)
  {
    latest_control_cmd_ = msg;
  }

  void control_cmd_timer_callback()
  {
    if (latest_control_cmd_ != nullptr) {
      control_cmd_pub_->publish(*latest_control_cmd_);
    }
  }

  rclcpp::Subscription<Control>::SharedPtr control_cmd_sub_;
  rclcpp::Publisher<Control>::SharedPtr control_cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_cmd_timer_;
  Control::SharedPtr latest_control_cmd_;
  int control_cmd_rate_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicRateController>());
  rclcpp::shutdown();
  return 0;
}