#include "whill_interface/whill_interface.hpp"
#include <cmath>

WhillInterface::WhillInterface() : Node("whill_interface")
{
  get_parameters();

  battery_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/status/battery_charge", 1);
  velocity_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 1);
  steering_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 1);
  gear_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 1);

  whill_sub_ = this->create_subscription<whill_msgs::msg::ModelCr2State>(
      "/whill/states/model_cr2", 1,
      std::bind(&WhillInterface::whill_callback, this, std::placeholders::_1)
  );
}

void WhillInterface::get_parameters() {
  this->get_parameter("wheel_tread", wheel_tread_);
  this->get_parameter("motor_speed_factor", motor_speed_factor_);
  this->get_parameter("min_velocity_threshold", min_velocity_threshold_);
  this->get_parameter("max_velocity_threshold", max_velocity_threshold_);
  this->get_parameter("min_steering_threshold", min_steering_threshold_);
  this->get_parameter("max_steering_threshold", max_steering_threshold_);
}

void WhillInterface::whill_callback(const whill_msgs::msg::ModelCr2State::SharedPtr msg) {
    // battery
    auto battery_msg = tier4_vehicle_msgs::msg::BatteryStatus();
    battery_msg.stamp = this->get_clock()->now();
    battery_msg.energy_level = msg->battery_power;
    battery_pub_->publish(battery_msg);

    // velocity
    auto velocity_msgs = autoware_vehicle_msgs::msg::VelocityReport();
    velocity_msgs.header.stamp = this->get_clock()->now();
    velocity_msgs.header.frame_id = "base_link";

    double right_speed = -1 * msg->right_motor_speed * motor_speed_factor_; // 速度の向きを合わせる
    double left_speed = msg->left_motor_speed * motor_speed_factor_;

    double longitudinal_velocity = (left_speed + right_speed) / 2;

    if (std::abs(longitudinal_velocity) < min_velocity_threshold_) {
        longitudinal_velocity = 0.0;
    }

    if (longitudinal_velocity > max_velocity_threshold_) {
        longitudinal_velocity = max_velocity_threshold_;
    }

    double heading_rate;
    if (right_speed == 0 && left_speed == 0) {
        heading_rate = 0.0;
    } else {
        heading_rate = (right_speed - left_speed) / wheel_tread_;
    }

    double turning_radius;
    if (heading_rate != 0) {
        turning_radius = longitudinal_velocity / heading_rate;
    } else {
        turning_radius = std::numeric_limits<double>::infinity();
    }

    double steering_angle = std::atan(wheel_tread_ / turning_radius);

    if (std::abs(steering_angle) > max_steering_threshold_) {
        steering_angle = std::copysign(max_steering_threshold_, steering_angle);
    }

    if (std::abs(steering_angle) < min_steering_threshold_) {
        steering_angle = 0.0;
    }

    velocity_msgs.longitudinal_velocity = longitudinal_velocity;
    velocity_msgs.lateral_velocity = 0.0;
    velocity_msgs.heading_rate = heading_rate;
    velocity_pub_->publish(velocity_msgs);

    // steering
    auto steering_msgs = autoware_vehicle_msgs::msg::SteeringReport();
    steering_msgs.stamp = this->get_clock()->now();
    steering_msgs.steering_tire_angle = steering_angle;
    steering_pub_->publish(steering_msgs);

    // gear
    auto gear_msgs = autoware_vehicle_msgs::msg::GearReport();
    gear_msgs.stamp = this->get_clock()->now();

    if (longitudinal_velocity == 0.0 && steering_angle == 0.0) {
        gear_msgs.report = autoware_vehicle_msgs::msg::GearReport::PARK;
    } else {
        gear_msgs.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
    }

    gear_pub_->publish(gear_msgs);
    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhillInterface>());
    rclcpp::shutdown();

    return 0;
}