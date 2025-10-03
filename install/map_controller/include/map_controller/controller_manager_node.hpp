#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "map_controller/map_controller.hpp"

namespace map_controller
{
class ControllerManagerNode : public rclcpp::Node
{
public:
  explicit ControllerManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void initialiseController();

  void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void onTimer();
  void publishVisualisation(const ControllerOutput & command);

  std::size_t findNearestWaypointIndex(const VehicleState & state) const;
  double computeSignedLateralError(const VehicleState & state,
                                   const Waypoint & waypoint) const;
  void publishDriveCommand(const ControllerOutput & command);
  void updateWaypointMetadata(std::vector<Waypoint> & path);
  void populatePathMessage(nav_msgs::msg::Path & msg);
  bool loadWaypointsFromCsv(const std::string & csv_path);

  MapControllerParams params_{};
  std::unique_ptr<MapController> controller_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleState current_state_{};
  bool has_state_{false};
  std::vector<Waypoint> current_path_;
  bool has_path_{false};
  bool path_dirty_{false};

  double track_length_m_{0.0};
  std::string state_topic_{};
  std::string path_topic_{};
  std::string csv_file_path_{};
  std::string map_frame_id_{"map"};
  double default_waypoint_speed_mps_{5.0};
  std::chrono::milliseconds timer_period_{50};
  DrivingMode driving_mode_{DrivingMode::Racing};
  bool publish_visualisation_{true};

  double previous_speed_mps_{0.0};
  rclcpp::Time previous_state_stamp_{};
};
}  // namespace map_controller
