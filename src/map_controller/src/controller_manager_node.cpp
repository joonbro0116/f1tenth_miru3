#include "map_controller/controller_manager_node.hpp"

namespace map_controller
{
ControllerManagerNode::ControllerManagerNode()
: rclcpp::Node("map_controller_manager")
{
  params_ = MapControllerParams{};
  controller_ = std::make_unique<MapController>(params_);

  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControllerManagerNode::onTimer, this));
}

void ControllerManagerNode::onTimer()
{
  VehicleState state{};
  std::vector<Waypoint> path;
  ControllerOutput cmd = controller_->computeCommand(state, path);

  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.drive.speed = static_cast<float>(cmd.target_speed_mps);
  drive_msg.drive.acceleration = static_cast<float>(cmd.target_accel_mps2);
  drive_msg.drive.steering_angle = static_cast<float>(cmd.steering_angle_rad);
  drive_pub_->publish(drive_msg);
}
}  // namespace map_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(map_controller::ControllerManagerNode)
