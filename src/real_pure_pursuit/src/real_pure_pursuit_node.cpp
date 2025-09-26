#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <limits>
#include <chrono>
#include <algorithm>
#include <iomanip>

struct Waypoint { double x, y, theta; };

class RealPurePursuitNode : public rclcpp::Node {
public:
    RealPurePursuitNode()
    : Node("real_pure_pursuit_node"),
      speed_error_integral_(0.0), prev_speed_error_(0.0), prev_commanded_speed_(0.0),
      current_x_(0.0), current_y_(0.0), current_yaw_(0.0), current_speed_(0.0)
    {
        // ===== Parameters =====
        declare_parameter<std::string>("csv_file_path", "/home/f1/f1tenth_ws/joon_path_generate/raceline/jg_thursday.csv");
        declare_parameter<std::string>("path_frame_id", "map");
        declare_parameter<std::string>("planned_path_topic", "/planned_path");
        declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
        declare_parameter<std::string>("drive_topic", "/vesc/low_level/ackermann_cmd_mux/output");

        declare_parameter<double>("lookahead_distance", 1.0);
        declare_parameter<double>("wheelbase", 0.3302);
        declare_parameter<double>("command_speed", 2.0);
        declare_parameter<int>("publish_path_period_ms", 500);
        declare_parameter<int>("control_period_ms", 50);

        // PID parameters for speed control
        declare_parameter<double>("speed_kp", 0.3);
        declare_parameter<double>("speed_ki", 0.01);
        declare_parameter<double>("speed_kd", 0.08);
        declare_parameter<double>("max_speed", 5.0);
        declare_parameter<double>("min_speed", 0.5);
        declare_parameter<double>("target_speed_straight", 3.0);
        declare_parameter<double>("target_speed_curve", 1.5);
        declare_parameter<double>("curvature_threshold", 0.2);
        declare_parameter<double>("speed_change_rate_limit", 1.0);

        get_parameter("csv_file_path", csv_path_);
        get_parameter("path_frame_id", path_frame_);
        get_parameter("planned_path_topic", planned_path_topic_);
        get_parameter("odom_topic", odom_topic_);
        get_parameter("drive_topic", drive_topic_);

        get_parameter("lookahead_distance", lookahead_distance_);
        get_parameter("wheelbase", wheelbase_);
        get_parameter("command_speed", command_speed_);
        int pub_ms, ctrl_ms;
        get_parameter("publish_path_period_ms", pub_ms);
        get_parameter("control_period_ms", ctrl_ms);

        // Get PID parameters
        get_parameter("speed_kp", speed_kp_);
        get_parameter("speed_ki", speed_ki_);
        get_parameter("speed_kd", speed_kd_);
        get_parameter("max_speed", max_speed_);
        get_parameter("min_speed", min_speed_);
        get_parameter("target_speed_straight", target_speed_straight_);
        get_parameter("target_speed_curve", target_speed_curve_);
        get_parameter("curvature_threshold", curvature_threshold_);
        get_parameter("speed_change_rate_limit", speed_change_rate_limit_);

        // Initialize PID state
        last_control_time_ = now();

        // ===== TF buffer/listener =====
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ===== Load CSV =====
        if (!load_csv()) {
            RCLCPP_FATAL(get_logger(), "CSV load failed: %s", csv_path_.c_str());
            throw std::runtime_error("CSV load failed");
        }
        RCLCPP_INFO(get_logger(), "CSV loaded successfully: %zu points (frame=%s)",
                    waypoints_map_.size(), path_frame_.c_str());

        // ===== Publishers/Subscribers =====
        path_pub_  = create_publisher<nav_msgs::msg::Path>(planned_path_topic_, 10);
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
        marker_pub_= create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&RealPurePursuitNode::odom_callback, this, std::placeholders::_1)
        );

        // ===== Timers =====
        path_timer_ = create_wall_timer(
            std::chrono::milliseconds(pub_ms),
            std::bind(&RealPurePursuitNode::publish_path_timer, this)
        );

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(ctrl_ms),
            std::bind(&RealPurePursuitNode::control_timer, this)
        );

        // ===== Initialize data logging =====
        auto now_time = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now_time);
        std::stringstream ss;
        ss << "/tmp/real_pure_pursuit_log_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
        log_filename_ = ss.str();

        log_file_.open(log_filename_);
        if (log_file_.is_open()) {
            // CSV header
            log_file_ << "timestamp,vehicle_x,vehicle_y,vehicle_yaw_deg,current_speed,target_speed,commanded_speed,"
                      << "lookahead_x,lookahead_y,lookahead_distance,curvature,steering_angle_deg,"
                      << "speed_error,closest_waypoint_idx,distance_to_closest\n";
            RCLCPP_INFO(get_logger(), "Logging data to: %s", log_filename_.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to open log file: %s", log_filename_.c_str());
        }

        RCLCPP_INFO(get_logger(), "Real Pure Pursuit node ready. odom=%s, drive=%s, planned_path=%s",
                    odom_topic_.c_str(), drive_topic_.c_str(), planned_path_topic_.c_str());
    }

    ~RealPurePursuitNode() {
        if (log_file_.is_open()) {
            log_file_.close();
            RCLCPP_INFO(get_logger(), "Log file closed: %s", log_filename_.c_str());
        }
    }

private:
    // ===== CSV load =====
    bool load_csv() {
        std::ifstream f(csv_path_);
        if (!f.is_open()) return false;

        std::string line;
        waypoints_map_.clear();
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string c0, c1, c2;
            if (!std::getline(ss, c0, ',')) continue;
            if (!std::getline(ss, c1, ',')) continue;
            if (!std::getline(ss, c2, ',')) c2 = "0";
            try {
                double x = std::stod(c0);
                double y = std::stod(c1);
                double th = std::stod(c2);
                waypoints_map_.push_back({x, y, th});
            } catch(...) { continue; }
        }
        return !waypoints_map_.empty();
    }

    // ===== Publish /planned_path =====
    void publish_path_timer() {
        if (waypoints_map_.empty()) return;
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = path_frame_;

        path.poses.reserve(waypoints_map_.size());
        for (const auto& wp : waypoints_map_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = wp.x;
            ps.pose.position.y = wp.y;
            ps.pose.orientation.w = 1.0;
            path.poses.push_back(ps);
        }
        path_pub_->publish(path);
    }

    // ===== Odom callback =====
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        const auto &q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        current_speed_ = msg->twist.twist.linear.x;
    }

    // ===== Main control loop =====
    void control_timer() {
        if (waypoints_map_.empty()) return;

        // Find lookahead point (forward direction only)
        bool found = false;
        double lx=0, ly=0;
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        // First find closest point
        for (size_t i=0; i<waypoints_map_.size(); ++i) {
            double dx = waypoints_map_[i].x - current_x_;
            double dy = waypoints_map_[i].y - current_y_;
            double dist = std::hypot(dx, dy);

            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // Find lookahead point from closest point forward
        for (size_t i=closest_idx; i<waypoints_map_.size(); ++i) {
            double dx = waypoints_map_[i].x - current_x_;
            double dy = waypoints_map_[i].y - current_y_;
            double dist = std::hypot(dx, dy);

            // Check if it's in front of vehicle (local x > 0)
            double local_x = std::cos(-current_yaw_) * dx - std::sin(-current_yaw_) * dy;

            if (local_x > 0.1 && dist >= lookahead_distance_) {
                lx = waypoints_map_[i].x;
                ly = waypoints_map_[i].y;
                found = true;
                RCLCPP_DEBUG(get_logger(), "Found lookahead at idx=%zu, dist=%.2f, local_x=%.2f", i, dist, local_x);
                break;
            }
        }
        if (!found) {
            // If not found forward, use point ahead of closest
            size_t next_idx = std::min(closest_idx + 5, waypoints_map_.size() - 1);
            lx = waypoints_map_[next_idx].x;
            ly = waypoints_map_[next_idx].y;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "No forward lookahead found, using waypoint %zu. Closest: idx=%zu, dist=%.2f",
                next_idx, closest_idx, min_dist);
        }

        // Debug info
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Vehicle: (%.2f,%.2f,%.2f°) | Lookahead: (%.2f,%.2f) | Closest: idx=%zu,dist=%.2f",
            current_x_, current_y_, current_yaw_*180/M_PI, lx, ly, closest_idx, min_dist);

        // Transform to vehicle local coordinates
        double dx = lx - current_x_;
        double dy = ly - current_y_;
        double cx =  std::cos(-current_yaw_) * dx - std::sin(-current_yaw_) * dy;
        double cy =  std::sin(-current_yaw_) * dx + std::cos(-current_yaw_) * dy;

        // Local coordinate debugging
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Global dx=%.2f, dy=%.2f | Local cx=%.2f, cy=%.2f",
            dx, dy, cx, cy);

        if (cx <= 0.05) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Lookahead behind/too close: cx=%.2f, cy=%.2f, global_dist=%.2f",
                cx, cy, std::hypot(dx, dy));
            return;
        }

        // Pure Pursuit
        double Ld = std::max(lookahead_distance_, std::hypot(cx, cy));
        double curvature = 2.0 * cy / (Ld * Ld);
        double steer = std::atan(wheelbase_ * curvature);
        double steer_clamped = std::clamp(steer, -M_PI/6.0, M_PI/6.0);

        // ===== Longitudinal Control (PID Speed Control) =====
        double target_speed = calculate_target_speed(curvature);
        double commanded_speed = calculate_pid_speed(target_speed, current_speed_);

        // Control calculation debugging
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Control: Ld=%.2f, curvature=%.3f, steer=%.1f°, target_spd=%.2f, curr_spd=%.2f, cmd_spd=%.2f",
            Ld, curvature, steer_clamped*180/M_PI, target_speed, current_speed_, commanded_speed);

        // Publish command
        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp = now();
        cmd.header.frame_id = "base_link";  // Real vehicle frame
        cmd.drive.speed = commanded_speed;
        cmd.drive.steering_angle = steer_clamped;
        drive_pub_->publish(cmd);

        // ===== Data Logging =====
        if (log_file_.is_open()) {
            double timestamp = now().seconds();
            double speed_error = target_speed - current_speed_;
            double lookahead_distance = std::hypot(lx - current_x_, ly - current_y_);

            log_file_ << std::fixed << std::setprecision(6)
                      << timestamp << ","
                      << current_x_ << "," << current_y_ << "," << current_yaw_*180/M_PI << ","
                      << current_speed_ << "," << target_speed << "," << commanded_speed << ","
                      << lx << "," << ly << "," << lookahead_distance << ","
                      << curvature << "," << steer_clamped*180/M_PI << ","
                      << speed_error << "," << closest_idx << "," << min_dist << "\n";
            log_file_.flush();  // Real-time save
        }

        publish_lookahead_marker(lx, ly);
    }

    void publish_lookahead_marker(double x, double y) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = now();
        m.ns = "lookahead";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.3;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.3;
        m.color.a = 1.0; m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0;
        marker_pub_->publish(m);
    }

    // ===== Target speed determination =====
    double calculate_target_speed(double curvature) {
        double abs_curvature = std::abs(curvature);

        // Straight line vs curve decision
        if (abs_curvature < curvature_threshold_) {
            return target_speed_straight_;
        } else {
            // Reduce speed based on curvature (smoother reduction)
            double speed_factor = std::max(0.7, 1.0 - (abs_curvature - curvature_threshold_) * 0.8);
            return target_speed_curve_ * speed_factor;
        }
    }

    // ===== PID speed control =====
    double calculate_pid_speed(double target_speed, double current_speed) {
        rclcpp::Time current_time = now();
        double dt = (current_time - last_control_time_).seconds();

        if (dt <= 0.0 || dt > 0.2) {  // Reset if dt is invalid or too large
            speed_error_integral_ = 0.0;
            prev_speed_error_ = 0.0;
            last_control_time_ = current_time;
            return target_speed;
        }

        // Calculate error
        double error = target_speed - current_speed;

        // Integral term (with windup protection)
        speed_error_integral_ += error * dt;
        speed_error_integral_ = std::clamp(speed_error_integral_, -10.0, 10.0);

        // Derivative term
        double error_derivative = (error - prev_speed_error_) / dt;

        // PID calculation
        double pid_output = speed_kp_ * error +
                           speed_ki_ * speed_error_integral_ +
                           speed_kd_ * error_derivative;

        // Apply PID output to current speed
        double commanded_speed = current_speed + pid_output;

        // Apply speed change rate limiting for smoother transitions
        if (prev_commanded_speed_ != 0.0) {
            double max_speed_change = speed_change_rate_limit_ * dt;
            double speed_diff = commanded_speed - prev_commanded_speed_;
            speed_diff = std::clamp(speed_diff, -max_speed_change, max_speed_change);
            commanded_speed = prev_commanded_speed_ + speed_diff;
        }

        // Clamp to limits
        commanded_speed = std::clamp(commanded_speed, min_speed_, max_speed_);

        // Update state
        prev_speed_error_ = error;
        prev_commanded_speed_ = commanded_speed;
        last_control_time_ = current_time;

        return commanded_speed;
    }

private:
    // Params
    std::string csv_path_, path_frame_, planned_path_topic_, odom_topic_, drive_topic_;
    double lookahead_distance_, wheelbase_, command_speed_;

    // PID control parameters
    double speed_kp_, speed_ki_, speed_kd_;
    double max_speed_, min_speed_;
    double target_speed_straight_, target_speed_curve_, curvature_threshold_;
    double speed_change_rate_limit_;

    // PID state variables
    double speed_error_integral_;
    double prev_speed_error_;
    double prev_commanded_speed_;
    rclcpp::Time last_control_time_;

    // Data
    std::vector<Waypoint> waypoints_map_;

    // State
    double current_x_, current_y_, current_yaw_, current_speed_;

    // ROS I/F
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr path_timer_, control_timer_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Data logging
    std::ofstream log_file_;
    std::string log_filename_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealPurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}