#pragma once

#include <memory>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "map_controller/map_controller.hpp"

namespace map_controller
{
class ControllerManagerNode : public rclcpp::Node
{
public:
  ControllerManagerNode();

private:
  void onTimer();

  MapControllerParams params_{};
  std::unique_ptr<MapController> controller_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace map_controller
