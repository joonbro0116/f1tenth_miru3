#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <memory>
#include <limits>
#include <queue>
#include <fstream>
#include <string>

class PathFollowNode : public rclcpp::Node
{
public:
    PathFollowNode() : Node("path_follow_node"), current_x_(0.0), current_y_(0.0), current_yaw_(0.0), current_speed_(0.0)
    {
        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        racing_line_pub_ = this->create_publisher<nav_msgs::msg::Path>("/racing_line", 10);
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PathFollowNode::lidar_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PathFollowNode::odom_callback, this, std::placeholders::_1));

        // Parameters for centerline extraction
        this->declare_parameter("map_pgm_file", "");
        this->declare_parameter("map_resolution", 0.05);
        this->declare_parameter("map_origin_x", 0.0);
        this->declare_parameter("map_origin_y", 0.0);
        this->declare_parameter("track_width", 2.0);
        this->declare_parameter("safety_margin", 0.3);
        this->declare_parameter("boundary_erosion", 3);

        // Parameters for pure pursuit
        this->declare_parameter("lookahead_distance", 1.5);
        this->declare_parameter("max_speed", 3.0);
        this->declare_parameter("min_speed", 0.5);
        this->declare_parameter("wheelbase", 0.3302);
        this->declare_parameter("obstacle_detection_distance", 2.0);

        // Load parameters
        std::string pgm_file = this->get_parameter("map_pgm_file").as_string();
        map_resolution_ = this->get_parameter("map_resolution").as_double();
        map_origin_x_ = this->get_parameter("map_origin_x").as_double();
        map_origin_y_ = this->get_parameter("map_origin_y").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        safety_margin_ = this->get_parameter("safety_margin").as_double();
        boundary_erosion_ = this->get_parameter("boundary_erosion").as_int();

        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        obstacle_detection_distance_ = this->get_parameter("obstacle_detection_distance").as_double();

        // Load centerline from CSV if available, otherwise extract from PGM
        if (loadCenterlineFromCSV("/home/f1/f1tenth_ws/maps/centerline.csv")) {
            RCLCPP_INFO(this->get_logger(), "중심선 CSV 로드 완료: %zu개 포인트", centerline_waypoints_.size());
            publishCenterlineAsPath();
        } else if (!pgm_file.empty()) {
            RCLCPP_INFO(this->get_logger(), "CSV 파일이 없어 PGM에서 중심선 추출 시작...");
            extractCenterlineFromPGM(pgm_file);
        } else {
            RCLCPP_ERROR(this->get_logger(), "PGM 파일 경로가 설정되지 않았습니다.");
        }

        current_waypoint_index_ = 0;
    }

private:
    struct Point2D {
        double x, y;
        Point2D(double x = 0, double y = 0) : x(x), y(y) {}
        Point2D operator+(const Point2D& other) const { return Point2D(x + other.x, y + other.y); }
        Point2D operator-(const Point2D& other) const { return Point2D(x - other.x, y - other.y); }
        Point2D operator*(double scalar) const { return Point2D(x * scalar, y * scalar); }
        double norm() const { return std::sqrt(x*x + y*y); }
        double dot(const Point2D& other) const { return x*other.x + y*other.y; }
    };

    struct GridPoint {
        int x, y;
        GridPoint(int x = 0, int y = 0) : x(x), y(y) {}
        bool operator==(const GridPoint& other) const { return x == other.x && y == other.y; }
    };

    // ROS publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr racing_line_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Map parameters
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    double track_width_;
    double safety_margin_;
    int boundary_erosion_;
    int map_width_;
    int map_height_;
    std::vector<uint8_t> map_pixels_;
    std::vector<std::vector<bool>> track_interior_mask_;

    // Pure pursuit parameters
    double lookahead_distance_;
    double max_speed_;
    double min_speed_;
    double wheelbase_;
    double obstacle_detection_distance_;

    // Vehicle state
    double current_x_, current_y_, current_yaw_, current_speed_;

    // Waypoints
    std::vector<Point2D> centerline_waypoints_;
    size_t current_waypoint_index_;

    // Gap follow parameters
    const int MIN_BUBBLE_SIZE = 10;
    const int MAX_BUBBLE_SIZE = 50;
    const double SAFETY_MARGIN = 0.5;

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
        current_x_ = odom_msg->pose.pose.position.x;
        current_y_ = odom_msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        auto q = odom_msg->pose.pose.orientation;
        current_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        
        // Extract speed
        current_speed_ = sqrt(pow(odom_msg->twist.twist.linear.x, 2) + 
                             pow(odom_msg->twist.twist.linear.y, 2));
    }

    bool loadCenterlineFromCSV(const std::string& csv_path) {
        std::ifstream csv_file(csv_path);
        if (!csv_file.is_open()) {
            return false;
        }

        centerline_waypoints_.clear();
        std::string line;
        bool first_line = true;

        while (std::getline(csv_file, line)) {
            if (first_line) {
                first_line = false;
                continue; // Skip header
            }

            size_t comma_pos = line.find(',');
            if (comma_pos == std::string::npos) continue;

            try {
                double x = std::stod(line.substr(0, comma_pos));
                double y = std::stod(line.substr(comma_pos + 1));
                centerline_waypoints_.emplace_back(x, y);
            } catch (const std::exception&) {
                continue;
            }
        }

        csv_file.close();
        return !centerline_waypoints_.empty();
    }

    void extractCenterlineFromPGM(const std::string& pgm_path) {
        // Load PGM file (simplified version of the original code)
        cv::Mat img = cv::imread(pgm_path, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "PGM 파일을 읽을 수 없습니다: %s", pgm_path.c_str());
            return;
        }

        map_width_ = img.cols;
        map_height_ = img.rows;
        map_pixels_.resize(map_width_ * map_height_);

        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                map_pixels_[y * map_width_ + x] = img.at<uint8_t>(y, x);
            }
        }

        RCLCPP_INFO(this->get_logger(), "PGM 맵 로드 완료: %dx%d", map_width_, map_height_);

        // Extract centerline (simplified implementation)
        if (detectTrackBoundary()) {
            createTrackInteriorMask();
            extractTrackCenterline();
            saveCenterlineToCSV();
        }
    }

    Point2D findLookaheadPoint() {
        if (centerline_waypoints_.empty()) {
            return Point2D(current_x_ + lookahead_distance_ * cos(current_yaw_),
                          current_y_ + lookahead_distance_ * sin(current_yaw_));
        }

        // Find the closest waypoint
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_idx = current_waypoint_index_;

        for (size_t i = 0; i < centerline_waypoints_.size(); ++i) {
            double dx = centerline_waypoints_[i].x - current_x_;
            double dy = centerline_waypoints_[i].y - current_y_;
            double distance = sqrt(dx * dx + dy * dy);

            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }

        // Find lookahead point
        size_t lookahead_idx = closest_idx;
        for (size_t i = closest_idx; i < centerline_waypoints_.size(); ++i) {
            double dx = centerline_waypoints_[i].x - current_x_;
            double dy = centerline_waypoints_[i].y - current_y_;
            double distance = sqrt(dx * dx + dy * dy);

            if (distance >= lookahead_distance_) {
                lookahead_idx = i;
                break;
            }
        }

        current_waypoint_index_ = lookahead_idx;
        return centerline_waypoints_[lookahead_idx];
    }

    double calculateSteeringAngle(const Point2D& target_point) {
        double dx = target_point.x - current_x_;
        double dy = target_point.y - current_y_;
        double target_angle = atan2(dy, dx);
        double angle_diff = target_angle - current_yaw_;

        // Normalize angle difference
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        double distance = sqrt(dx * dx + dy * dy);
        double curvature = 2.0 * sin(angle_diff) / distance;

        return atan(curvature * wheelbase_);
    }

    bool detectObstacle(const std::vector<float>& ranges) {
        for (const auto& range : ranges) {
            if (range > 0.0f && range < obstacle_detection_distance_) {
                return true;
            }
        }
        return false;
    }

    std::vector<float> preprocessLidar(const std::vector<float>& ranges) {
        std::vector<float> filtered_ranges(ranges.size());
        for (size_t i = 0; i < ranges.size(); ++i) {
            filtered_ranges[i] = (ranges[i] > 3.0f) ? 3.0f : ranges[i];
        }

        // Use front 180 degrees (left 90 + right 90)
        const int start_idx = 540 - 360;
        const int end_idx = 540 + 360;
        std::vector<float> cropped_ranges(filtered_ranges.begin() + start_idx, 
                                         filtered_ranges.begin() + end_idx + 1);
        
        return cropped_ranges;
    }

    std::pair<int, int> findMaxGap(const std::vector<float>& ranges) {
        int n = ranges.size();
        std::vector<float> temp = ranges;

        // Find nearest obstacle and create bubble
        int nearest_idx = -1;
        float nearest_dist = std::numeric_limits<float>::infinity();
        for (int i = 0; i < n; ++i) {
            if (temp[i] <= 0.0f || std::isinf(temp[i])) continue;
            if (temp[i] < nearest_dist) {
                nearest_dist = temp[i];
                nearest_idx = i;
            }
        }

        // Create adaptive bubble
        if (nearest_idx >= 0) {
            int bubble_size = std::max(MIN_BUBBLE_SIZE, 
                static_cast<int>(MIN_BUBBLE_SIZE + 
                (MAX_BUBBLE_SIZE - MIN_BUBBLE_SIZE) * 
                std::max(0.0, (2.0 - nearest_dist) / 2.0)));
            
            int start_bubble = std::max(0, nearest_idx - bubble_size);
            int end_bubble = std::min(n - 1, nearest_idx + bubble_size);
            for (int i = start_bubble; i <= end_bubble; ++i) {
                temp[i] = 0.0f;
            }
        }

        // Find maximum gap
        int current_gap_start = -1;
        int max_gap_start = -1;
        int max_gap_length = 0;

        for (int i = 0; i < n; ++i) {
            if (temp[i] > 0.0f) {
                if (current_gap_start < 0) {
                    current_gap_start = i;
                }
            } else {
                if (current_gap_start >= 0) {
                    int gap_length = i - current_gap_start;
                    if (gap_length > max_gap_length) {
                        max_gap_length = gap_length;
                        max_gap_start = current_gap_start;
                    }
                    current_gap_start = -1;
                }
            }
        }

        // Handle last gap
        if (current_gap_start >= 0) {
            int gap_length = n - current_gap_start;
            if (gap_length > max_gap_length) {
                max_gap_length = gap_length;
                max_gap_start = current_gap_start;
            }
        }

        return std::make_pair(max_gap_start, max_gap_length);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        if (centerline_waypoints_.empty()) {
            RCLCPP_WARN(this->get_logger(), "중심선 웨이포인트가 없습니다.");
            return;
        }

        std::vector<float> raw_ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());
        auto processed_ranges = preprocessLidar(raw_ranges);
        
        bool obstacle_detected = detectObstacle(processed_ranges);
        
        double steering_angle = 0.0;
        double speed = max_speed_;
        std::string mode = "";

        if (obstacle_detected) {
            // Gap follow mode
            auto [max_gap_start, max_gap_length] = findMaxGap(processed_ranges);
            
            if (max_gap_start >= 0 && max_gap_length > 0) {
                int best_point = max_gap_start + max_gap_length / 2;
                int center_idx = processed_ranges.size() / 2;
                int offset_idx = best_point - center_idx;
                steering_angle = offset_idx * scan_msg->angle_increment;
                speed = min_speed_;
                mode = "GAP_FOLLOW";
                
                RCLCPP_INFO(this->get_logger(), 
                    "GAP_FOLLOW: best_point=%d, steering=%.3f", best_point, steering_angle);
            } else {
                // Stop if no gap found
                speed = 0.0;
                steering_angle = 0.0;
                mode = "EMERGENCY_STOP";
            }
        } else {
            // Pure pursuit mode - follow centerline
            Point2D lookahead_point = findLookaheadPoint();
            steering_angle = calculateSteeringAngle(lookahead_point);
            speed = max_speed_;
            mode = "PURE_PURSUIT";
            
            RCLCPP_INFO(this->get_logger(),
                "PURE_PURSUIT: target=(%.2f, %.2f), current=(%.2f, %.2f), waypoint_idx=%zu",
                lookahead_point.x, lookahead_point.y, current_x_, current_y_, current_waypoint_index_);
        }

        // Limit steering angle
        const double max_steering = M_PI / 6.0; // 30 degrees
        steering_angle = std::max(-max_steering, std::min(max_steering, steering_angle));

        // Publish drive command
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header = scan_msg->header;
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle;
        drive_pub_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(),
            "Mode: %s, Steering: %.3f rad, Speed: %.1f m/s",
            mode.c_str(), steering_angle, speed);
    }

    void publishCenterlineAsPath() {
        if (centerline_waypoints_.empty()) return;

        nav_msgs::msg::Path centerline_msg;
        centerline_msg.header.stamp = this->get_clock()->now();
        centerline_msg.header.frame_id = "map";

        for (const auto& point : centerline_waypoints_) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = centerline_msg.header;
            pose_stamped.pose.position.x = point.x;
            pose_stamped.pose.position.y = point.y;
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;
            centerline_msg.poses.push_back(pose_stamped);
        }

        racing_line_pub_->publish(centerline_msg);
        RCLCPP_INFO(this->get_logger(), "중심선 퍼블리시 완료: %zu개 포인트", centerline_waypoints_.size());
    }

    // Simplified centerline extraction methods (placeholder implementations)
    bool detectTrackBoundary() { return false; }
    void createTrackInteriorMask() {}
    void extractTrackCenterline() {}
    void saveCenterlineToCSV() {}
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}