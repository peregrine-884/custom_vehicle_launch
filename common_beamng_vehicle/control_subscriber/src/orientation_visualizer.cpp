#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>

class ImuOrientationVisualizer : public rclcpp::Node
{
public:
    ImuOrientationVisualizer()
    : Node("imu_orientation_visualizer")
    {
        // IMUデータの購読
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&ImuOrientationVisualizer::imuCallback, this, std::placeholders::_1));

        // 矢印の可視化用パブリッシャー
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("imu_orientation_marker", 10);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        visualization_msgs::msg::Marker arrow_marker;
        
        arrow_marker.header.frame_id = "base_link";  // フレームIDを適宜変更
        arrow_marker.header.stamp = this->now();
        arrow_marker.ns = "imu_arrow";
        arrow_marker.id = 0;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        // 矢印の位置を設定
        arrow_marker.pose.position.x = 0.0;
        arrow_marker.pose.position.y = 0.0;
        arrow_marker.pose.position.z = 0.0;

        // IMUのオリエンテーションを取得し、そのまま使用
        tf2::Quaternion imu_orientation(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        // 結果を矢印のオリエンテーションに設定
        arrow_marker.pose.orientation.x = imu_orientation.x();
        arrow_marker.pose.orientation.y = imu_orientation.y();
        arrow_marker.pose.orientation.z = imu_orientation.z();
        arrow_marker.pose.orientation.w = imu_orientation.w();

        // 矢印のサイズを設定
        arrow_marker.scale.x = 2.0;  // 矢印の長さ
        arrow_marker.scale.y = 0.5;  // 矢印の幅
        arrow_marker.scale.z = 0.5;  // 矢印の厚み

        // 矢印の色を設定
        arrow_marker.color.r = 1.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 0.0;
        arrow_marker.color.a = 1.0;

        // マーカーの寿命を1秒に設定
        arrow_marker.lifetime = rclcpp::Duration(1, 0);

        // マーカーをパブリッシュ
        marker_publisher_->publish(arrow_marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOrientationVisualizer>());
    rclcpp::shutdown();
    return 0;
}
