#include "map_controller/controller_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <utility>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace map_controller
{
namespace
{
constexpr double kEpsilon = 1e-6;
}

ControllerManagerNode::ControllerManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("map_controller_manager", options)
{
  declareParameters();
  initialiseController();

  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pure_pursuit_path", 10);
  lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/lookahead_point", 10);

  state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    state_topic_, rclcpp::QoS{10}.best_effort(),
    std::bind(&ControllerManagerNode::stateCallback, this, std::placeholders::_1));

  if (!path_topic_.empty()) {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS{1}.best_effort(),
      std::bind(&ControllerManagerNode::pathCallback, this, std::placeholders::_1));
  }

  if (!csv_file_path_.empty()) {
    if (loadWaypointsFromCsv(csv_file_path_)) {
      RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s",
                  current_path_.size(), csv_file_path_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to load waypoints from %s", csv_file_path_.c_str());
    }
  }

  timer_ = this->create_wall_timer(
    timer_period_,
    std::bind(&ControllerManagerNode::onTimer, this));
}

void ControllerManagerNode::declareParameters()
{
  params_.t_clip_min = this->declare_parameter("t_clip_min", params_.t_clip_min);
  params_.t_clip_max = this->declare_parameter("t_clip_max", params_.t_clip_max);
  params_.m_l1 = this->declare_parameter("m_l1", params_.m_l1);
  params_.q_l1 = this->declare_parameter("q_l1", params_.q_l1);
  params_.speed_lookahead_s = this->declare_parameter("speed_lookahead_s", params_.speed_lookahead_s);
  params_.lat_err_coeff = this->declare_parameter("lat_err_coeff", params_.lat_err_coeff);
  params_.acc_scaler_for_steer = this->declare_parameter("acc_scaler_for_steer", params_.acc_scaler_for_steer);
  params_.dec_scaler_for_steer = this->declare_parameter("dec_scaler_for_steer", params_.dec_scaler_for_steer);
  params_.start_scale_speed_mps = this->declare_parameter("start_scale_speed_mps", params_.start_scale_speed_mps);
  params_.end_scale_speed_mps = this->declare_parameter("end_scale_speed_mps", params_.end_scale_speed_mps);
  params_.downscale_factor = this->declare_parameter("downscale_factor", params_.downscale_factor);
  params_.speed_lookahead_for_steer_s = this->declare_parameter(
    "speed_lookahead_for_steer_s", params_.speed_lookahead_for_steer_s);
  params_.prioritize_dyn = this->declare_parameter("prioritize_dyn", params_.prioritize_dyn);
  params_.trailing_gap_m = this->declare_parameter("trailing_gap_m", params_.trailing_gap_m);
  params_.trailing_p_gain = this->declare_parameter("trailing_p_gain", params_.trailing_p_gain);
  params_.trailing_i_gain = this->declare_parameter("trailing_i_gain", params_.trailing_i_gain);
  params_.trailing_d_gain = this->declare_parameter("trailing_d_gain", params_.trailing_d_gain);
  params_.blind_trailing_speed_mps = this->declare_parameter(
    "blind_trailing_speed_mps", params_.blind_trailing_speed_mps);
  params_.loop_rate_hz = this->declare_parameter("loop_rate_hz", params_.loop_rate_hz);
  params_.steering_lut_name = this->declare_parameter("steering_lut_name", params_.steering_lut_name);
  params_.state_machine_rate_hz = this->declare_parameter(
    "state_machine_rate_hz", params_.state_machine_rate_hz);
  params_.steering_change_threshold_rad = this->declare_parameter(
    "steering_change_threshold_rad", params_.steering_change_threshold_rad);

  track_length_m_ = this->declare_parameter("track_length_m", track_length_m_);
  state_topic_ = this->declare_parameter("state_topic", std::string("/odom"));
  path_topic_ = this->declare_parameter("path_topic", std::string("/local_waypoints"));
  csv_file_path_ = this->declare_parameter("csv_file_path", csv_file_path_);
  map_frame_id_ = this->declare_parameter("map_frame_id", map_frame_id_);
  default_waypoint_speed_mps_ = this->declare_parameter(
    "default_waypoint_speed_mps", default_waypoint_speed_mps_);
  publish_visualisation_ = this->declare_parameter("publish_visualisation", publish_visualisation_);

  const int timer_period_ms_default = static_cast<int>(timer_period_.count());
  const int timer_period_ms = this->declare_parameter("timer_period_ms", timer_period_ms_default);
  timer_period_ = std::chrono::milliseconds(std::max(1, timer_period_ms));

  const std::string driving_mode = this->declare_parameter("driving_mode", std::string("racing"));
  driving_mode_ = (driving_mode == "trailing") ? DrivingMode::Trailing : DrivingMode::Racing;
}

void ControllerManagerNode::initialiseController()
{
  controller_ = std::make_unique<MapController>(params_);
  controller_->setDrivingMode(driving_mode_);
  controller_->setTrackLength(track_length_m_);
}

void ControllerManagerNode::stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_state_.x_m = msg->pose.pose.position.x;
  current_state_.y_m = msg->pose.pose.position.y;
  current_state_.yaw_rad = tf2::getYaw(msg->pose.pose.orientation);

  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;
  current_state_.speed_mps = std::hypot(vx, vy);
  current_state_.frenet_speed_mps = current_state_.speed_mps;

  const rclcpp::Time stamp = msg->header.stamp;
  if (previous_state_stamp_.nanoseconds() > 0) {
    const double dt = (stamp - previous_state_stamp_).seconds();
    if (dt > kEpsilon) {
      current_state_.acceleration_mps2 = (current_state_.speed_mps - previous_speed_mps_) / dt;
    }
  }

  previous_state_stamp_ = stamp;
  previous_speed_mps_ = current_state_.speed_mps;

  has_state_ = true;
}

void ControllerManagerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_.clear();
  current_path_.reserve(msg->poses.size());

  for (std::size_t i = 0; i < msg->poses.size(); ++i) {
    const auto & pose = msg->poses[i].pose;
    Waypoint waypoint{};
    waypoint.x_m = pose.position.x;
    waypoint.y_m = pose.position.y;
    waypoint.target_speed_mps = default_waypoint_speed_mps_;
    current_path_.push_back(waypoint);
  }

  updateWaypointMetadata(current_path_);
  has_path_ = !current_path_.empty();
  path_dirty_ = has_path_;
}

void ControllerManagerNode::onTimer()
{
  if (!controller_) {
    return;
  }

  if (!has_state_ || !has_path_) {
    return;
  }

  VehicleState state = current_state_;

  const std::size_t nearest_index = findNearestWaypointIndex(state);
  const Waypoint & nearest = current_path_.at(nearest_index);
  state.frenet_d_m = computeSignedLateralError(state, nearest);
  state.frenet_s_m = nearest.frenet_s;

  controller_->setDrivingMode(driving_mode_);
  controller_->setTrackLength(track_length_m_);

  const auto command = controller_->computeCommand(state, current_path_);
  publishDriveCommand(command);
  publishVisualisation(command);
}

std::size_t ControllerManagerNode::findNearestWaypointIndex(const VehicleState & state) const
{
  double best_distance_sq = std::numeric_limits<double>::max();
  std::size_t best_index = 0;

  for (std::size_t i = 0; i < current_path_.size(); ++i) {
    const double dx = state.x_m - current_path_[i].x_m;
    const double dy = state.y_m - current_path_[i].y_m;
    const double distance_sq = dx * dx + dy * dy;

    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_index = i;
    }
  }

  return best_index;
}

double ControllerManagerNode::computeSignedLateralError(const VehicleState & state,
                                                         const Waypoint & waypoint) const
{
  const double dx = state.x_m - waypoint.x_m;
  const double dy = state.y_m - waypoint.y_m;
  return (-std::sin(waypoint.heading_rad) * dx) + (std::cos(waypoint.heading_rad) * dy);
}

void ControllerManagerNode::publishVisualisation(const ControllerOutput & command)
{
  if (!publish_visualisation_) {
    return;
  }

  if (path_pub_ && has_path_ && path_dirty_) {
    nav_msgs::msg::Path path_msg;
    populatePathMessage(path_msg);
    path_pub_->publish(path_msg);
    path_dirty_ = false;
  }

  if (lookahead_pub_ && has_path_) {
    geometry_msgs::msg::PoseStamped lookahead_msg;
    lookahead_msg.header.stamp = this->now();
    lookahead_msg.header.frame_id = map_frame_id_;
    lookahead_msg.pose.position.x = command.lookahead_point[0];
    lookahead_msg.pose.position.y = command.lookahead_point[1];
    lookahead_msg.pose.orientation.w = 1.0;
    lookahead_pub_->publish(lookahead_msg);
  }
}

void ControllerManagerNode::updateWaypointMetadata(std::vector<Waypoint> & path)
{
  if (path.empty()) {
    return;
  }

  double accumulated_s = 0.0;
  double previous_heading = 0.0;

  for (std::size_t i = 0; i < path.size(); ++i) {
    double heading = previous_heading;
    if (i + 1 < path.size()) {
      const double dx = path[i + 1].x_m - path[i].x_m;
      const double dy = path[i + 1].y_m - path[i].y_m;
      heading = std::atan2(dy, dx);
    }

    path[i].heading_rad = heading;
    path[i].frenet_s = accumulated_s;
    path[i].frenet_d = 0.0;
    path[i].curvature_radpm = 0.0;
    path[i].longitudinal_accel_mps2 = 0.0;

    if (i + 1 < path.size()) {
      accumulated_s += std::hypot(path[i + 1].x_m - path[i].x_m,
                                  path[i + 1].y_m - path[i].y_m);
      previous_heading = heading;
    }
  }
}

void ControllerManagerNode::populatePathMessage(nav_msgs::msg::Path & msg)
{
  msg.header.stamp = this->now();
  msg.header.frame_id = map_frame_id_;
  msg.poses.clear();
  msg.poses.reserve(current_path_.size());

  for (const auto & waypoint : current_path_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg.header.stamp;
    pose.header.frame_id = map_frame_id_;
    pose.pose.position.x = waypoint.x_m;
    pose.pose.position.y = waypoint.y_m;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
}

bool ControllerManagerNode::loadWaypointsFromCsv(const std::string & csv_path)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Unable to open waypoint CSV: %s", csv_path.c_str());
    return false;
  }

  std::vector<Waypoint> loaded_path;
  std::string line;
  bool first_line = true;
  bool has_frenet_s_column = false;
  bool has_heading_column = false;

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    if (first_line) {
      first_line = false;
      const bool has_alpha = std::any_of(line.begin(), line.end(), [](unsigned char c) {
        return std::isalpha(c);
      });
      if (has_alpha) {
        continue;
      }
    }

    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> tokens;
    while (std::getline(ss, cell, ',')) {
      auto start = cell.find_first_not_of(" \t");
      auto end = cell.find_last_not_of(" \t");
      if (start == std::string::npos) {
        tokens.emplace_back();
      } else {
        tokens.emplace_back(cell.substr(start, end - start + 1));
      }
    }

    if (tokens.size() < 2) {
      continue;
    }

    try {
      Waypoint waypoint{};
      waypoint.x_m = std::stod(tokens[0]);
      waypoint.y_m = std::stod(tokens[1]);
      waypoint.target_speed_mps = (tokens.size() > 2 && !tokens[2].empty())
                                    ? std::stod(tokens[2])
                                    : default_waypoint_speed_mps_;

      if (tokens.size() > 3 && !tokens[3].empty()) {
        waypoint.frenet_s = std::stod(tokens[3]);
        has_frenet_s_column = true;
      }
      if (tokens.size() > 4 && !tokens[4].empty()) {
        waypoint.frenet_d = std::stod(tokens[4]);
      }
      if (tokens.size() > 5 && !tokens[5].empty()) {
        waypoint.curvature_radpm = std::stod(tokens[5]);
      }
      if (tokens.size() > 6 && !tokens[6].empty()) {
        waypoint.heading_rad = std::stod(tokens[6]);
        has_heading_column = true;
      }
      if (tokens.size() > 7 && !tokens[7].empty()) {
        waypoint.longitudinal_accel_mps2 = std::stod(tokens[7]);
      }

      loaded_path.push_back(waypoint);
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Skipping CSV line due to parse error: %s", e.what());
      continue;
    }
  }

  file.close();

  if (loaded_path.empty()) {
    return false;
  }

  if (!has_frenet_s_column || !has_heading_column) {
    updateWaypointMetadata(loaded_path);
  }
  current_path_ = std::move(loaded_path);
  has_path_ = true;
  path_dirty_ = true;

  return true;
}

void ControllerManagerNode::publishDriveCommand(const ControllerOutput & command)
{
  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = this->now();
  drive_msg.header.frame_id = "base_link";
  drive_msg.drive.speed = static_cast<float>(command.target_speed_mps);
  drive_msg.drive.acceleration = static_cast<float>(command.target_accel_mps2);
  drive_msg.drive.steering_angle = static_cast<float>(command.steering_angle_rad);
  drive_pub_->publish(drive_msg);
}
}  // namespace map_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(map_controller::ControllerManagerNode)
