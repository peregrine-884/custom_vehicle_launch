#include "whill_controller/whill_controller.hpp"

WhillController::WhillController() : Node("whill_controller")
{
  get_parameters();
  speed_limit_factor_ = 1.0;

  autoware_sub_ = this->create_subscription<Control>(
    "/control/command/control_cmd", 1,
    std::bind(&WhillController::autoware_callback, this, std::placeholders::_1)
  );

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 1, std::bind(&WhillController::joy_callback, this, std::placeholders::_1)
  );

  joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
    "/whill/controller/joy", 1
  );

  control_mode_pub_ = this->create_publisher<ControlModeReport>(
    "/vehicle/status/control_mode", 1
  );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&WhillController::timer_callback, this));
}

void WhillController::get_parameters()
{
  this->get_parameter("min_speed", min_speed_);
  this->get_parameter("max_speed", max_speed_);
  this->get_parameter("min_angle", min_angle_);
  this->get_parameter("max_angle", max_angle_);
  this->get_parameter("normalizer", normalizer_);
  this->get_parameter("manual_button", manual_button_);
  this->get_parameter("speed_plus_button", speed_plus_button_);
  this->get_parameter("speed_minus_button", speed_minus_button_);
  this->get_parameter("speed_axes", speed_axes_);
  this->get_parameter("angle_axes", angle_axes_);

  // RCLCPP_INFO(this->get_logger(), "Parameters:");
  // RCLCPP_INFO(this->get_logger(), "min_speed: %f", min_speed_);
  // RCLCPP_INFO(this->get_logger(), "max_speed: %f", max_speed_);
  // RCLCPP_INFO(this->get_logger(), "min_angle: %f", min_angle_);
  // RCLCPP_INFO(this->get_logger(), "max_angle: %f", max_angle_);
  // RCLCPP_INFO(this->get_logger(), "normalizer: %f", normalizer_);
  // RCLCPP_INFO(this->get_logger(), "manual_button: %d", manual_button_);
  // RCLCPP_INFO(this->get_logger(), "speed_plus_button: %d", speed_plus_button_);
  // RCLCPP_INFO(this->get_logger(), "speed_minus_button: %d", speed_minus_button_);
  // RCLCPP_INFO(this->get_logger(), "speed_axes: %d", speed_axes_);
  // RCLCPP_INFO(this->get_logger(), "angle_axes: %d", angle_axes_);
}

void WhillController::autoware_callback(const Control::SharedPtr msg)
{
  if (manual_flag_) {
    return;
  }

  float speed = msg->longitudinal.velocity;
  float angle = msg->lateral.steering_tire_angle;

  if (speed < min_speed_) {
      speed = min_speed_;
  } else if (speed > max_speed_) {
      speed = max_speed_;
  }

  if (angle < min_angle_) {
      angle = min_angle_;
  } else if (angle > max_angle_) {
      angle = max_angle_;
  }

  float normalized_speed = speed / max_speed_;
  float normalized_angle = angle * 2.0;

  sensor_msgs::msg::Joy pub_msg;
  pub_msg.header.stamp = this->now();
  pub_msg.header.frame_id = "joy"; 

  pub_msg.axes.resize(2);

  pub_msg.axes[1] = normalized_speed;
  pub_msg.axes[0] = normalized_angle;

  joy_pub_->publish(pub_msg);
}

void WhillController::joy_callback(const sensor_msgs::msg::Joy & msg)
{
  auto control_msg = msg;

  if (control_msg.buttons[speed_plus_button_]) plus_ = true;
  if (control_msg.buttons[speed_minus_button_]) minus_ = true;
  if (control_msg.buttons[speed_plus_button_] == 1 && plus_) {
    speed_limit_factor_ += 0.1;
    plus_ = false;
  }
  if (control_msg.buttons[speed_minus_button_] == 1 && minus_) {
    speed_limit_factor_ -= 0.1;
    minus_ = false;
  }

  if (speed_limit_factor_ > 1) speed_limit_factor_ = 1;
  if (speed_limit_factor_ < 0) speed_limit_factor_ = 0;

  if (control_msg.buttons[manual_button_] == 1) {
    control_msg.axes[1] = control_msg.axes[speed_axes_] * speed_limit_factor_;
    control_msg.axes[0] = control_msg.axes[angle_axes_] * speed_limit_factor_;
    manual_flag_ = true;
    joy_pub_->publish(control_msg);
  } else {
    manual_flag_ = false;
  }

}

void WhillController::timer_callback()
{
  // 1: autonomous
  // 4: manual
  if (manual_flag_) {
    publish_control_mode(ControlModeReport::MANUAL);
  } else {
    publish_control_mode(ControlModeReport::AUTONOMOUS);
  }
}

void WhillController::publish_control_mode(uint8_t mode)
{
  auto control_mode_msg = ControlModeReport();
  control_mode_msg.stamp = this->get_clock()->now();
  control_mode_msg.mode = mode;

  control_mode_pub_->publish(control_mode_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhillController>());
  rclcpp::shutdown();
  return 0;
}