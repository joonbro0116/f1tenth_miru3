#include <rclcpp/rclcpp.hpp>

#include "map_controller/controller_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<map_controller::ControllerManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
