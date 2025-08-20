#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanFrameRewriter : public rclcpp::Node {
public:
  ScanFrameRewriter()
  : Node("scan_frame_rewriter"),
    target_rate_hz_(declare_parameter<double>("target_rate", 40.0)),
    output_frame_(declare_parameter<std::string>("output_frame", "laser")),
    input_topic_(declare_parameter<std::string>("input_topic", "/scan")),
    output_topic_(declare_parameter<std::string>("output_topic", "/scan_fix"))
  {
    using rclcpp::SensorDataQoS;
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, SensorDataQoS(),
      std::bind(&ScanFrameRewriter::cb, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);
    min_period_ = rclcpp::Duration::from_seconds(1.0 / target_rate_hz_);
    RCLCPP_INFO(get_logger(), "Rewriting %s -> %s @ %.1f Hz (frame=%s)",
                input_topic_.c_str(), output_topic_.c_str(),
                target_rate_hz_, output_frame_.c_str());
  }

private:
  void cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto now = this->now();
    if (last_pub_time_.nanoseconds() != 0 &&
        (now - last_pub_time_) < min_period_) return;

    sensor_msgs::msg::LaserScan out = *msg;
    out.header.frame_id = output_frame_;
    pub_->publish(out);
    last_pub_time_ = now;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  std::string input_topic_, output_topic_, output_frame_;
  double target_rate_hz_;
  rclcpp::Time last_pub_time_;
  rclcpp::Duration min_period_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFrameRewriter>());
  rclcpp::shutdown();
  return 0;
}
