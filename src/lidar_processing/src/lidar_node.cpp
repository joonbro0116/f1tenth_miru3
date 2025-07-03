// src/lidar_node.cpp

#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarNode : public rclcpp::Node {
public:
  LidarNode()
  : Node("lidar_node")
  {
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LidarNode::scanCallback, this, std::placeholders::_1)
    );
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan/processed", 10);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // ±90도 범위의 인덱스 계산
    float min_deg = -90.0f;
    float max_deg =  90.0f;
    float min_rad = min_deg * M_PI / 180.0f;
    float max_rad = max_deg * M_PI / 180.0f;

    int start_idx = std::ceil((min_rad - msg->angle_min) / msg->angle_increment);
    int end_idx   = std::floor((max_rad - msg->angle_min) / msg->angle_increment);

    // 인덱스 범위 보정
    start_idx = std::max(0, start_idx);
    end_idx = std::min(static_cast<int>(msg->ranges.size()) - 1, end_idx);

    // 새 메시지 생성
    auto out = std::make_shared<sensor_msgs::msg::LaserScan>();
    *out = *msg;
    out->ranges.assign(msg->ranges.begin() + start_idx, msg->ranges.begin() + end_idx + 1);
    if (!msg->intensities.empty()) {
      out->intensities.assign(msg->intensities.begin() + start_idx, msg->intensities.begin() + end_idx + 1);
    }
    out->angle_min = msg->angle_min + start_idx * msg->angle_increment;
    out->angle_max = msg->angle_min + end_idx * msg->angle_increment;

    pub_->publish(*out);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
