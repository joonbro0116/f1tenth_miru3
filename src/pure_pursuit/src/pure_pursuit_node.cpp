#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

struct Waypoint {
    double x, y, target_speed;
    Waypoint(double x = 0.0, double y = 0.0, double speed = 1.0) : x(x), y(y), target_speed(speed) {}
};

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double max_integral = 10.0, double max_output = 100.0)
        : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral), max_output_(max_output),
          previous_error_(0.0), integral_(0.0), previous_time_(0.0) {}

    double compute(double setpoint, double measured_value, double current_time) {
        double error = setpoint - measured_value;
        double dt = current_time - previous_time_;

        if (dt <= 0.0) {
            return 0.0; // Avoid division by zero
        }

        // Proportional term
        double proportional = kp_ * error;

        // Integral term with windup protection
        integral_ += error * dt;
        integral_ = std::max(-max_integral_, std::min(integral_, max_integral_));
        double integral_term = ki_ * integral_;

        // Derivative term
        double derivative = kd_ * (error - previous_error_) / dt;

        // Compute output
        double output = proportional + integral_term + derivative;

        // Limit output
        output = std::max(-max_output_, std::min(output, max_output_));

        // Store for next iteration
        previous_error_ = error;
        previous_time_ = current_time;

        return output;
    }

    void reset() {
        previous_error_ = 0.0;
        integral_ = 0.0;
        previous_time_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double max_integral_, max_output_;
    double previous_error_, integral_, previous_time_;
};

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_, this, false)
    {
        // Parameters
        this->declare_parameter("csv_file_path", "/home/f1/f1tenth_ws/maps_racelines/raceline/fuck_jg_1_mintime_vmax3ms.csv");
        this->declare_parameter("lookahead_distance", 1.2);
        this->declare_parameter("max_speed", 8.0);
        this->declare_parameter("min_speed", 1.0);
        this->declare_parameter("max_steering_angle", 0.4189);  // 24 degrees in radians
        this->declare_parameter("wheelbase", 0.33);  // F1TENTH wheelbase

        // PID parameters for speed control
        this->declare_parameter("speed_kp", 1.0);
        this->declare_parameter("speed_ki", 0.1);
        this->declare_parameter("speed_kd", 0.05);
        this->declare_parameter("speed_max_integral", 2.0);
        this->declare_parameter("speed_max_output", 5.0);

        // Curvature-based speed control parameters
        this->declare_parameter("enable_curvature_control", true);
        this->declare_parameter("max_curvature", 2.0);  // Maximum allowable curvature (1/m)
        this->declare_parameter("min_curve_speed", 2.0);  // Minimum speed in tight curves (m/s)
        this->declare_parameter("curvature_speed_factor", 0.7);  // Speed reduction factor for curves
        this->declare_parameter("curve_lookahead_points", 5);  // Points ahead to check for curves

        // Adaptive lookahead and steering smoothing parameters
        this->declare_parameter("enable_adaptive_lookahead", true);
        this->declare_parameter("min_lookahead_distance", 0.8);
        this->declare_parameter("max_lookahead_distance", 2.5);
        this->declare_parameter("lookahead_speed_factor", 0.3);
        this->declare_parameter("steering_smoothing_factor", 0.7);

        csv_file_path_ = this->get_parameter("csv_file_path").as_string();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();

        // Initialize PID controller
        double speed_kp = this->get_parameter("speed_kp").as_double();
        double speed_ki = this->get_parameter("speed_ki").as_double();
        double speed_kd = this->get_parameter("speed_kd").as_double();
        double speed_max_integral = this->get_parameter("speed_max_integral").as_double();
        double speed_max_output = this->get_parameter("speed_max_output").as_double();

        speed_pid_ = std::make_unique<PIDController>(speed_kp, speed_ki, speed_kd, speed_max_integral, speed_max_output);

        // Initialize curvature control parameters
        enable_curvature_control_ = this->get_parameter("enable_curvature_control").as_bool();
        max_curvature_ = this->get_parameter("max_curvature").as_double();
        min_curve_speed_ = this->get_parameter("min_curve_speed").as_double();
        curvature_speed_factor_ = this->get_parameter("curvature_speed_factor").as_double();
        curve_lookahead_points_ = this->get_parameter("curve_lookahead_points").as_int();

        // Initialize adaptive lookahead and steering smoothing
        enable_adaptive_lookahead_ = this->get_parameter("enable_adaptive_lookahead").as_bool();
        min_lookahead_distance_ = this->get_parameter("min_lookahead_distance").as_double();
        max_lookahead_distance_ = this->get_parameter("max_lookahead_distance").as_double();
        lookahead_speed_factor_ = this->get_parameter("lookahead_speed_factor").as_double();
        steering_smoothing_factor_ = this->get_parameter("steering_smoothing_factor").as_double();

        previous_steering_angle_ = 0.0;

        // Load waypoints from CSV
        if (!loadWaypoints()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from CSV file: %s", csv_file_path_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from CSV file", waypoints_.size());

        // Publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pure_pursuit_path", 10);
        lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/lookahead_point", 10);

        // Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&PurePursuitNode::poseCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

        // Timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&PurePursuitNode::controlLoop, this));

        // Publish path for visualization
        publishPath();

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node initialized successfully!");
        RCLCPP_INFO(this->get_logger(), "Waiting for localization data on /amcl_pose...");
    }

private:
    // ROS2 components
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    std::string csv_file_path_;
    double lookahead_distance_;
    double max_speed_;
    double min_speed_;
    double max_steering_angle_;
    double wheelbase_;

    // State variables
    std::vector<Waypoint> waypoints_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
    bool pose_received_ = false;
    size_t closest_waypoint_index_ = 0;

    // PID controller and speed feedback
    std::unique_ptr<PIDController> speed_pid_;
    double current_speed_ = 0.0;

    // Curvature-based speed control
    bool enable_curvature_control_;
    double max_curvature_;
    double min_curve_speed_;
    double curvature_speed_factor_;
    int curve_lookahead_points_;

    // Adaptive lookahead and steering smoothing
    bool enable_adaptive_lookahead_;
    double min_lookahead_distance_;
    double max_lookahead_distance_;
    double lookahead_speed_factor_;
    double steering_smoothing_factor_;
    double previous_steering_angle_;

    bool loadWaypoints() {
        waypoints_.clear();
        std::ifstream file(csv_file_path_);

        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file: %s", csv_file_path_.c_str());
            return false;
        }

        std::string line;
        bool first_line = true;

        while (std::getline(file, line)) {
            if (first_line) {
                first_line = false;
                continue; // Skip header line
            }

            std::stringstream ss(line);
            std::string x_str, y_str;

            std::string speed_str;
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, speed_str, ',')) {
                try {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    double speed = std::stod(speed_str);
                    waypoints_.emplace_back(x, y, speed);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Error parsing line: %s, error: %s", line.c_str(), e.what());
                }
            } else if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
                // Fallback for CSV without speed column
                try {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    waypoints_.emplace_back(x, y, max_speed_); // Use max_speed as default
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Error parsing line: %s, error: %s", line.c_str(), e.what());
                }
            }
        }

        file.close();
        return !waypoints_.empty();
    }

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract speed from odometry
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        current_speed_ = sqrt(vx * vx + vy * vy);
    }

    void publishPath() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->get_clock()->now();

        for (const auto& wp : waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->get_clock()->now();
            pose.pose.position.x = wp.x;
            pose.pose.position.y = wp.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }

    double distance(const Waypoint& a, const Waypoint& b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    size_t findClosestWaypoint(double x, double y) {
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < waypoints_.size(); ++i) {
            double dist = distance(Waypoint(x, y), waypoints_[i]);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        return closest_idx;
    }

    double getAdaptiveLookaheadDistance() {
        if (!enable_adaptive_lookahead_) {
            return lookahead_distance_;
        }

        // Adaptive lookahead based on speed
        double adaptive_distance = lookahead_distance_ + (current_speed_ * lookahead_speed_factor_);

        // Clamp to min/max values
        return std::max(min_lookahead_distance_, std::min(adaptive_distance, max_lookahead_distance_));
    }

    Waypoint findLookaheadPoint(double x, double y, double yaw) {
        size_t start_idx = findClosestWaypoint(x, y);
        closest_waypoint_index_ = start_idx;

        double current_lookahead = getAdaptiveLookaheadDistance();

        // Search for lookahead point starting from closest waypoint
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            size_t idx = (start_idx + i) % waypoints_.size();
            double dist = distance(Waypoint(x, y), waypoints_[idx]);

            if (dist >= current_lookahead) {
                // Check if this point is in front of the vehicle
                double dx = waypoints_[idx].x - x;
                double dy = waypoints_[idx].y - y;
                double angle_to_point = atan2(dy, dx);
                double angle_diff = angle_to_point - yaw;

                // Normalize angle difference
                while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

                // If point is in front (within 90 degrees), use it
                if (abs(angle_diff) < M_PI/2) {
                    return waypoints_[idx];
                }
            }
        }

        // Fallback: return a point ahead of closest waypoint
        size_t ahead_idx = (start_idx + 10) % waypoints_.size();
        return waypoints_[ahead_idx];
    }

    double calculateSteeringAngle(double x, double y, double yaw, const Waypoint& lookahead_point) {
        // Transform lookahead point to vehicle coordinate frame
        double dx = lookahead_point.x - x;
        double dy = lookahead_point.y - y;

        // Rotate to vehicle frame
        double local_x = cos(yaw) * dx + sin(yaw) * dy;
        double local_y = -sin(yaw) * dx + cos(yaw) * dy;

        // Pure pursuit steering angle calculation
        double lookahead_dist = sqrt(local_x * local_x + local_y * local_y);

        if (lookahead_dist < 0.1) {
            return previous_steering_angle_; // Return previous angle to avoid sudden changes
        }

        double curvature = 2.0 * local_y / (lookahead_dist * lookahead_dist);
        double raw_steering_angle = atan(wheelbase_ * curvature);

        // Apply steering smoothing filter
        double steering_angle = steering_smoothing_factor_ * previous_steering_angle_ +
                               (1.0 - steering_smoothing_factor_) * raw_steering_angle;

        // Limit steering angle
        steering_angle = std::max(-max_steering_angle_, std::min(steering_angle, max_steering_angle_));

        // Store for next iteration
        previous_steering_angle_ = steering_angle;

        return steering_angle;
    }

    double calculateCurvature(size_t waypoint_idx) {
        if (waypoints_.size() < 3 || waypoint_idx == 0 || waypoint_idx >= waypoints_.size() - 1) {
            return 0.0;
        }

        // Use three consecutive points to calculate curvature
        const auto& p1 = waypoints_[waypoint_idx - 1];
        const auto& p2 = waypoints_[waypoint_idx];
        const auto& p3 = waypoints_[waypoint_idx + 1];

        // Calculate vectors
        double dx1 = p2.x - p1.x;
        double dy1 = p2.y - p1.y;
        double dx2 = p3.x - p2.x;
        double dy2 = p3.y - p2.y;

        // Calculate angle change
        double angle1 = atan2(dy1, dx1);
        double angle2 = atan2(dy2, dx2);
        double angle_diff = angle2 - angle1;

        // Normalize angle difference
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // Calculate distance between points
        double dist = sqrt(dx1 * dx1 + dy1 * dy1) + sqrt(dx2 * dx2 + dy2 * dy2);

        if (dist < 0.01) {
            return 0.0;
        }

        // Curvature = angle change / arc length
        return abs(angle_diff) / (dist * 0.5);
    }

    double getMaxCurvatureAhead(size_t start_idx) {
        double max_curvature = 0.0;

        for (int i = 0; i < curve_lookahead_points_ && (start_idx + i) < waypoints_.size(); ++i) {
            size_t idx = start_idx + i;
            double curvature = calculateCurvature(idx);
            max_curvature = std::max(max_curvature, curvature);
        }

        return max_curvature;
    }

    double getTargetSpeed() {
        // Get target speed from current waypoint
        if (waypoints_.empty()) {
            return min_speed_;
        }

        // Use the target speed from the closest waypoint
        double target_speed = waypoints_[closest_waypoint_index_].target_speed;

        // Apply curvature-based speed control if enabled
        if (enable_curvature_control_) {
            double max_curve_ahead = getMaxCurvatureAhead(closest_waypoint_index_);

            if (max_curve_ahead > 0.1) {  // If there's significant curvature ahead
                // Calculate speed reduction based on curvature
                double curvature_ratio = std::min(max_curve_ahead / max_curvature_, 1.0);
                double speed_reduction = curvature_ratio * curvature_speed_factor_;

                // Apply speed reduction
                target_speed = target_speed * (1.0 - speed_reduction);

                // Ensure minimum curve speed
                target_speed = std::max(target_speed, min_curve_speed_);

                // Log curvature information
                if (max_curve_ahead > 0.5) {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000, // 2 seconds
                        "High curvature detected: %.3f, reducing speed to: %.2f m/s",
                        max_curve_ahead, target_speed
                    );
                }
            }
        }

        // Ensure target speed is within limits
        return std::max(min_speed_, std::min(target_speed, max_speed_));
    }

    double calculateSpeedWithPID(double target_speed) {
        // Get current time in seconds
        double current_time = this->get_clock()->now().seconds();

        // Compute PID output
        double speed_adjustment = speed_pid_->compute(target_speed, current_speed_, current_time);

        // Apply adjustment to target speed
        double commanded_speed = target_speed + speed_adjustment;

        // Ensure commanded speed is within limits
        return std::max(min_speed_, std::min(commanded_speed, max_speed_));
    }

    void controlLoop() {
        if (!pose_received_ || waypoints_.empty()) {
            return;
        }

        // Extract current position and orientation
        double x = current_pose_.pose.pose.position.x;
        double y = current_pose_.pose.pose.position.y;

        // Convert quaternion to yaw
        tf2::Quaternion quat;
        tf2::fromMsg(current_pose_.pose.pose.orientation, quat);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        // Find lookahead point
        Waypoint lookahead_point = findLookaheadPoint(x, y, yaw);

        // Calculate steering angle
        double steering_angle = calculateSteeringAngle(x, y, yaw, lookahead_point);

        // Get target speed and calculate actual speed using PID
        double target_speed = getTargetSpeed();
        double speed = calculateSpeedWithPID(target_speed);

        // Publish drive command
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->get_clock()->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle;

        drive_pub_->publish(drive_msg);

        // Publish lookahead point for visualization
        geometry_msgs::msg::PoseStamped lookahead_msg;
        lookahead_msg.header.frame_id = "map";
        lookahead_msg.header.stamp = this->get_clock()->now();
        lookahead_msg.pose.position.x = lookahead_point.x;
        lookahead_msg.pose.position.y = lookahead_point.y;
        lookahead_msg.pose.position.z = 0.0;
        lookahead_msg.pose.orientation.w = 1.0;

        lookahead_pub_->publish(lookahead_msg);

        // Log status
        // Calculate current curvature for logging
        double current_curvature = 0.0;
        if (enable_curvature_control_) {
            current_curvature = getMaxCurvatureAhead(closest_waypoint_index_);
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000, // 1 second
            "Position: (%.2f, %.2f), Yaw: %.2f, Steering: %.3f rad, Target Speed: %.2f, Current Speed: %.2f, Commanded Speed: %.2f, Curvature: %.3f, Closest WP: %zu",
            x, y, yaw, steering_angle, target_speed, current_speed_, speed, current_curvature, closest_waypoint_index_
        );
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();

    RCLCPP_INFO(node->get_logger(), "Starting Pure Pursuit Node for F1TENTH Autonomous Racing");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}