#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "map_controller/map_controller.hpp"

namespace map_controller
{

class MapControllerManagerNode : public rclcpp::Node
{
public:
  MapControllerManagerNode()
  : rclcpp::Node("map_controller_manager")
  {
    declareParameters();
    loadParameters();

    try {
      controller_ = std::make_unique<MapController>(params_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialise MAP controller: %s", e.what());
      throw;
    }

    // Create publishers FIRST before using them
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/map_controller/path", 10);
    lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/map_controller/lookahead", 10);

    // Load waypoints and publish path
    if (!csv_file_path_.empty()) {
      if (!loadWaypointsFromCsv(csv_file_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from %s", csv_file_path_.c_str());
      } else {
        publishGlobalPath();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());
      }
    }

    // Subscribe to AMCL for accurate position (map frame)
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&MapControllerManagerNode::amclCallback, this, std::placeholders::_1));

    // Subscribe to odometry for velocity
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20,
      std::bind(&MapControllerManagerNode::odomCallback, this, std::placeholders::_1));

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, loop_rate_hz_)));
    control_timer_ = this->create_wall_timer(period, std::bind(&MapControllerManagerNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "MAP Controller initialized - waiting for AMCL pose and odometry...");
  }

private:
  void declareParameters()
  {
    this->declare_parameter<std::string>("csv_file_path", "");
    this->declare_parameter<double>("t_clip_min", 0.8);
    this->declare_parameter<double>("t_clip_max", 5.0);
    this->declare_parameter<double>("m_l1", 0.6);
    this->declare_parameter<double>("q_l1", -0.18);
    this->declare_parameter<double>("speed_lookahead", 0.25);
    this->declare_parameter<double>("lat_err_coeff", 1.0);
    this->declare_parameter<double>("acc_scaler_for_steer", 1.2);
    this->declare_parameter<double>("dec_scaler_for_steer", 0.9);
    this->declare_parameter<double>("start_scale_speed", 7.0);
    this->declare_parameter<double>("end_scale_speed", 8.0);
    this->declare_parameter<double>("downscale_factor", 0.2);
    this->declare_parameter<double>("speed_lookahead_for_steer", 0.0);
    this->declare_parameter<double>("steering_change_threshold", 0.4);
    this->declare_parameter<std::string>("steering_lut", "NUC2_pacejka");
    this->declare_parameter<double>("loop_rate_hz", 40.0);
  }

  void loadParameters()
  {
    csv_file_path_ = this->get_parameter("csv_file_path").as_string();
    params_.t_clip_min = this->get_parameter("t_clip_min").as_double();
    params_.t_clip_max = this->get_parameter("t_clip_max").as_double();
    params_.m_l1 = this->get_parameter("m_l1").as_double();
    params_.q_l1 = this->get_parameter("q_l1").as_double();
    params_.speed_lookahead_s = this->get_parameter("speed_lookahead").as_double();
    params_.lat_err_coeff = this->get_parameter("lat_err_coeff").as_double();
    params_.acc_scaler_for_steer = this->get_parameter("acc_scaler_for_steer").as_double();
    params_.dec_scaler_for_steer = this->get_parameter("dec_scaler_for_steer").as_double();
    params_.start_scale_speed_mps = this->get_parameter("start_scale_speed").as_double();
    params_.end_scale_speed_mps = this->get_parameter("end_scale_speed").as_double();
    params_.downscale_factor = this->get_parameter("downscale_factor").as_double();
    params_.speed_lookahead_for_steer_s = this->get_parameter("speed_lookahead_for_steer").as_double();
    params_.steering_change_threshold_rad = this->get_parameter("steering_change_threshold").as_double();
    params_.steering_lut_name = this->get_parameter("steering_lut").as_string();
    loop_rate_hz_ = this->get_parameter("loop_rate_hz").as_double();
  }

  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const auto & pose = msg->pose.pose;

    // Update position from AMCL (accurate map frame localization)
    state_.x_m = pose.position.x;
    state_.y_m = pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    state_.yaw_rad = tf2::getYaw(q);

    has_amcl_ = true;

    if (!amcl_first_received_) {
      RCLCPP_INFO(this->get_logger(), "AMCL pose received - position: (%.2f, %.2f)",
                  state_.x_m, state_.y_m);
      amcl_first_received_ = true;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & twist = msg->twist.twist;

    // Extract velocity from odometry
    const double vx = twist.linear.x;
    const double vy = twist.linear.y;
    const double speed = std::sqrt(vx * vx + vy * vy);

    // Calculate acceleration
    const rclcpp::Time stamp = msg->header.stamp;
    if (has_odom_) {
      const double dt = (stamp - last_odom_stamp_).seconds();
      if (dt > 1e-3) {
        state_.acceleration_mps2 = (speed - state_.speed_mps) / dt;
      }
    }

    state_.speed_mps = speed;
    has_odom_ = true;
    last_odom_stamp_ = stamp;

    if (!odom_first_received_) {
      RCLCPP_INFO(this->get_logger(), "Odometry received - speed: %.2f m/s", speed);
      odom_first_received_ = true;
    }
  }

  void controlLoop()
  {
    // Require both AMCL (position) and odometry (velocity) before controlling
    if (!has_amcl_ || !has_odom_ || waypoints_.size() < 2) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Control loop waiting: has_amcl=%d, has_odom=%d, waypoints=%zu",
                           has_amcl_, has_odom_, waypoints_.size());
      return;
    }

    const auto output = controller_->computeCommand(state_, waypoints_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Control: speed=%.2f, steering=%.3f",
                        output.target_speed_mps, output.steering_angle_rad);

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = "map";
    drive_msg.drive.speed = output.target_speed_mps;
    drive_msg.drive.acceleration = output.target_accel_mps2;
    drive_msg.drive.steering_angle = output.steering_angle_rad;
    drive_pub_->publish(drive_msg);

    geometry_msgs::msg::PoseStamped lookahead_msg;
    lookahead_msg.header = drive_msg.header;
    lookahead_msg.pose.position.x = output.lookahead_point[0];
    lookahead_msg.pose.position.y = output.lookahead_point[1];
    lookahead_msg.pose.orientation.w = 1.0;
    lookahead_pub_->publish(lookahead_msg);

    if (path_pub_->get_subscription_count() > 0) {
      publishGlobalPath();
    }
  }

  bool loadWaypointsFromCsv(const std::string & file_path)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open CSV file: %s", file_path.c_str());
      return false;
    }

    std::vector<Waypoint> waypoints;
    std::string line;
    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }
      std::stringstream ss(line);
      std::string token;
      std::vector<double> values;
      while (std::getline(ss, token, ',')) {
        try {
          values.push_back(std::stod(token));
        } catch (const std::exception &) {
          values.push_back(0.0);
        }
      }
      if (values.size() < 2) {
        continue;
      }
      Waypoint wp;
      wp.x_m = values[0];
      wp.y_m = values[1];
      wp.target_speed_mps = values.size() > 2 ? values[2] : 3.0;
      waypoints.push_back(wp);
    }

    if (waypoints.size() < 2) {
      return false;
    }

    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
      const double dx = waypoints[i + 1].x_m - waypoints[i].x_m;
      const double dy = waypoints[i + 1].y_m - waypoints[i].y_m;
      waypoints[i].heading_rad = std::atan2(dy, dx);
    }
    waypoints.back().heading_rad = waypoints[waypoints.size() - 2].heading_rad;

    waypoints_ = std::move(waypoints);
    return true;
  }

  void publishGlobalPath()
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";
    path_msg.poses.reserve(waypoints_.size());
    for (const auto & wp : waypoints_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = wp.x_m;
      pose.pose.position.y = wp.y_m;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }
    path_pub_->publish(path_msg);
  }

  MapControllerParams params_{};
  std::unique_ptr<MapController> controller_;
  std::vector<Waypoint> waypoints_;
  std::string csv_file_path_;
  double loop_rate_hz_{40.0};

  VehicleState state_{};
  bool has_amcl_{false};
  bool has_odom_{false};
  bool amcl_first_received_{false};
  bool odom_first_received_{false};
  rclcpp::Time last_odom_stamp_{};

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace map_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<map_controller::MapControllerManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
