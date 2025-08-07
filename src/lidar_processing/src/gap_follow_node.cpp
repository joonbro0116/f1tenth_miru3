// 2025-07-23, adaptive bubble size, pure pursuit, gap follow
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

class GapFollow : public rclcpp::Node {
public:
    GapFollow()
    : Node("gap_follow_node"), current_x_(0.0), current_y_(0.0), current_yaw_(0.0), current_speed_(0.0)
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&GapFollow::lidar_callback, this, std::placeholders::_1)
        );
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&GapFollow::odom_callback, this, std::placeholders::_1)
        );
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10
        );
        
        initialize_waypoints();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    
    // Pure pursuit 관련 변수
    std::vector<geometry_msgs::msg::Point> dynamic_waypoints_;
    double current_x_, current_y_, current_yaw_;
    double current_speed_;
    const double LOOKAHEAD_DISTANCE = 2.0;
    const double OBSTACLE_DETECTION_DISTANCE = 2.0;
    const size_t MAX_WAYPOINTS = 10;
    
    // Adaptive bubble 파라미터
    const int MIN_BUBBLE_SIZE = 10;
    const int MAX_BUBBLE_SIZE = 50;
    const double SAFETY_MARGIN = 0.5;

    void initialize_waypoints() {
        dynamic_waypoints_.clear();
    }
    
    geometry_msgs::msg::Point create_point(double x, double y) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.0;
        return p;
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
        current_x_ = odom_msg->pose.pose.position.x;
        current_y_ = odom_msg->pose.pose.position.y;
        
        // 쿼터니언에서 yaw 추출
        auto q = odom_msg->pose.pose.orientation;
        current_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        
        // 속도 추출
        current_speed_ = sqrt(pow(odom_msg->twist.twist.linear.x, 2) + 
                             pow(odom_msg->twist.twist.linear.y, 2));
    }
    
    bool detect_obstacle(const std::vector<float>& ranges) {
        for (const auto& range : ranges) {
            if (range > 0.0f && range < OBSTACLE_DETECTION_DISTANCE) {
                return true;
            }
        }
        return false;
    }
    
    int calculate_adaptive_bubble_size(float nearest_distance, double angle_increment) {
        // 1. 거리 기반 계산: 가까울수록 큰 bubble
        double distance_factor = std::max(0.1, (3.0 - nearest_distance) / 3.0);
        
        // 2. 속도 기반 계산: 빠를수록 큰 bubble (제동거리 고려)
        double speed_factor = std::min(2.0, current_speed_ / 2.0);
        
        // 3. 각도 해상도 기반: LIDAR 해상도에 맞는 최소 크기
        double angular_size = SAFETY_MARGIN / nearest_distance; // 실제 safety margin 각도
        int angular_bubble_size = static_cast<int>(angular_size / angle_increment);
        
        // 4. 종합 계산
        int adaptive_size = static_cast<int>(MIN_BUBBLE_SIZE + 
                                           (MAX_BUBBLE_SIZE - MIN_BUBBLE_SIZE) * 
                                           distance_factor * (1.0 + speed_factor * 0.3));
        
        // 5. 각도 기반 최소값과 비교하여 더 큰 값 선택
        adaptive_size = std::max(adaptive_size, angular_bubble_size);
        
        // 6. 범위 제한
        return std::max(MIN_BUBBLE_SIZE, std::min(MAX_BUBBLE_SIZE, adaptive_size));
    }
    
    double pure_pursuit_steering(const geometry_msgs::msg::Point& target) {
        double dx = target.x - current_x_;
        double dy = target.y - current_y_;
        double target_angle = atan2(dy, dx);
        double angle_diff = target_angle - current_yaw_;
        
        // 각도 정규화 (-pi ~ pi)
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        
        double distance = sqrt(dx * dx + dy * dy);
        double curvature = 2.0 * sin(angle_diff) / distance;
        
        const double wheelbase = 0.3302; // F1TENTH 표준 wheelbase
        return atan(curvature * wheelbase);
    }
    
    void add_dynamic_waypoint(const geometry_msgs::msg::Point& point) {
        dynamic_waypoints_.push_back(point);
        
        // 최대 waypoint 수 제한
        if (dynamic_waypoints_.size() > MAX_WAYPOINTS) {
            dynamic_waypoints_.erase(dynamic_waypoints_.begin());
        }
    }
    
    geometry_msgs::msg::Point convert_best_point_to_world(int best_point_idx, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg) {
        // LIDAR 스캔에서 best point의 각도 계산
        int center_idx = 360; // processed_ranges는 720개 (좌우 360씩)
        int offset_idx = best_point_idx - center_idx;
        double angle = offset_idx * scan_msg->angle_increment;
        
        // 전방 일정 거리로 waypoint 생성 (2m 전방)
        double target_distance = 2.0;
        double world_x = current_x_ + target_distance * cos(current_yaw_ + angle);
        double world_y = current_y_ + target_distance * sin(current_yaw_ + angle);
        
        return create_point(world_x, world_y);
    }
    
    geometry_msgs::msg::Point get_lookahead_point() {
        if (dynamic_waypoints_.empty()) {
            // waypoint가 없으면 전방으로 직진
            return create_point(current_x_ + LOOKAHEAD_DISTANCE * cos(current_yaw_),
                              current_y_ + LOOKAHEAD_DISTANCE * sin(current_yaw_));
        }
        
        // 가장 최근 waypoint를 목표로 설정
        return dynamic_waypoints_.back();
    }

    std::vector<float> preprocess_lidar(const std::vector<float>& ranges) {
        // 3m 이상 range 처리
        std::vector<float> filtered_ranges(ranges.size());
        for (size_t i = 0; i < ranges.size(); ++i) {
            filtered_ranges[i] = (ranges[i] > 3.0f) ? 3.0f : ranges[i]; // infinity 대신 3.0f로 처리
        }
        
        // 정면 좌우 90도만 사용 (예: 1080개 중간이 540, 좌우 360씩)
        const int start_idx = 540 - 360;
        const int end_idx   = 540 + 360;
        std::vector<float> cropped_ranges(filtered_ranges.begin() + start_idx, filtered_ranges.begin() + end_idx + 1);
        
        return cropped_ranges;
    }

    std::pair<int, int> find_bubble_max_gap(const std::vector<float>& ranges, double angle_increment) {
        int n = ranges.size();
        std::vector<float> temp = ranges; // 복사본 생성
        
        // 가장 가까운 점을 찾는다
        int nearest_idx = -1;
        float nearest_dist = std::numeric_limits<float>::infinity();
        for (int i = 0; i < n; ++i) { // int i로 수정
            if (temp[i] <= 0.0f || std::isinf(temp[i])) continue; // 유효하지 않은 값 제외
            if (temp[i] < nearest_dist) {
                nearest_dist = temp[i];
                nearest_idx = i;
            }
        }
        
        // Adaptive 버블 생성
        if (nearest_idx >= 0) {
            int bubble_size = calculate_adaptive_bubble_size(nearest_dist, angle_increment);
            int start_bubble_idx = std::max(0, nearest_idx - bubble_size);
            int end_bubble_idx = std::min(n - 1, nearest_idx + bubble_size);
            for (int i = start_bubble_idx; i <= end_bubble_idx; ++i) {
                temp[i] = 0.0f; // 버블 영역을 0으로 설정
            }
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Adaptive bubble: distance=%.2f, speed=%.2f, size=%d", 
                nearest_dist, current_speed_, bubble_size);
        }
        
        // 최대 gap 찾기
        int current_gap_start = -1;
        int max_gap_start = -1;
        int max_gap_length = 0;
        
        for (int i = 0; i < n; ++i) { // int i로 수정
            if (temp[i] > 0.0f) { // 유효한 거리 (버블이 아닌 영역)
                if (current_gap_start < 0) {
                    current_gap_start = i; // gap 시작
                }
            } else {
                // gap이 끝났을 때
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
        
        // 마지막 gap 처리
        if (current_gap_start >= 0) {
            int gap_length = n - current_gap_start;
            if (gap_length > max_gap_length) {
                max_gap_length = gap_length;
                max_gap_start = current_gap_start;
            }
        }
        
        return std::make_pair(max_gap_start, max_gap_length);
    }

    int find_best_point(int max_gap_start, int max_gap_length) { // 매개변수 타입 추가
        if (max_gap_start < 0 || max_gap_length <= 0) {
            return -1; // 유효하지 않은 gap
        }
        int best_point = max_gap_start + max_gap_length / 2;
        return best_point;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        std::vector<float> raw_ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());
        auto processed_ranges = preprocess_lidar(raw_ranges);
        
        // 장애물 감지 여부 확인
        bool obstacle_detected = detect_obstacle(processed_ranges);
        
        double steering_angle = 0.0;
        double speed = 2.0;
        std::string mode = "";
        
        // 항상 gap follow로 best point 찾기
        auto [max_gap_start, max_gap_length] = find_bubble_max_gap(processed_ranges, scan_msg->angle_increment);
        int best_point_idx = find_best_point(max_gap_start, max_gap_length);
        
        if (best_point_idx < 0) {
            RCLCPP_WARN(this->get_logger(), "No valid gap found, stopping");
            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.header = scan_msg->header;
            drive_msg.drive.speed = 0.0f;
            drive_msg.drive.steering_angle = 0.0f;
            drive_pub_->publish(drive_msg);
            return;
        }
        
        // Best point를 world coordinate waypoint로 변환하여 추가
        auto new_waypoint = convert_best_point_to_world(best_point_idx, scan_msg);
        add_dynamic_waypoint(new_waypoint);
        
        if (obstacle_detected) {
            // Gap Follow 모드: 직접 조향
            int center_idx = processed_ranges.size() / 2;
            int offset_idx = best_point_idx - center_idx;
            steering_angle = offset_idx * scan_msg->angle_increment;
            
            speed = 1.5;
            mode = "GAP_FOLLOW";
            
            RCLCPP_INFO(this->get_logger(),
                "GAP_FOLLOW: center=%d, best=%d, offset=%d, gap_start=%d, gap_len=%d, waypoint=(%.2f, %.2f)",
                center_idx, best_point_idx, offset_idx, max_gap_start, max_gap_length,
                new_waypoint.x, new_waypoint.y);
        } else {
            // Pure Pursuit 모드: waypoint 추종
            auto target_point = get_lookahead_point();
            steering_angle = pure_pursuit_steering(target_point);
            speed = 0.8;
            mode = "PURE_PURSUIT";
            
            RCLCPP_INFO(this->get_logger(),
                "PURE_PURSUIT: target=(%.2f, %.2f), current=(%.2f, %.2f), waypoints=%lu",
                target_point.x, target_point.y, current_x_, current_y_, dynamic_waypoints_.size());
        }
        
        // 조향각 제한 (-30도 ~ +30도)
        const double max_steering = M_PI / 6.0; // 30도
        steering_angle = std::max(-max_steering, std::min(max_steering, steering_angle));
        
        RCLCPP_INFO(this->get_logger(),
            "Mode: %s, Steering: %.3f rad, Speed: %.1f m/s",
            mode.c_str(), steering_angle, speed);

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header = scan_msg->header;
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle;
        drive_pub_->publish(drive_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFollow>());
    rclcpp::shutdown();
    return 0;
}
