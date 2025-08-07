#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

using std::placeholders::_1;

class GapFollow : public rclcpp::Node {
public:
    GapFollow() : Node("gap_follow_pure_pursuit"), prev_goal_idx_(-1) {
        // 퍼블리셔
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        // 서브스크립션
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&GapFollow::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GapFollow::odom_callback, this, _1));

        prev_steering_angle_ = 0.0;
        current_speed_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "Gap Follow + Pure Pursuit (120deg FOV + Fallback) Started!");
    }

private:
    // ================= ODOM 콜백 =================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_speed_ = msg->twist.twist.linear.x;
    }

    // ================= LIDAR 콜백 =================
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int n = msg->ranges.size();
        if (n == 0) return;

        // 파라미터
        const float max_range = 7.0;
        const float disparity_threshold = 1.0;
        const float car_half_width = 0.15;
        const float margin = 0.3;
        const float wheel_base = 0.33;

        // 전방 좌우 60도씩 (총 120도)
        const float fov_half_angle = M_PI / 2.0; // 90도

        // ---------- Lidar 전처리 ----------
        std::vector<float> ranges = msg->ranges;
        for (int i = 0; i < n; i++) {
            float angle = msg->angle_min + i * msg->angle_increment;

            // 전방 120도만 사용 (-60도 ~ +60도)
            if (angle < -fov_half_angle || angle > fov_half_angle) {
                ranges[i] = 0.0;
                continue;
            }

            if (!std::isfinite(ranges[i])) ranges[i] = 0.0;
            else if (ranges[i] > max_range) ranges[i] = max_range;
        }

        // ---------- Gap 탐지 ----------
        int goal_idx = -1;
        bool is_curve = false;

        float max_disparity = -1.0;
        int boundary_idx = -1;
        float disparity_diff = 0.0;

        for (int i = 0; i < n - 1; i++) {
            if (ranges[i] <= 0.0 || ranges[i + 1] <= 0.0) continue;

            float diff = ranges[i + 1] - ranges[i];
            if (fabs(diff) > max_disparity) {
                max_disparity = fabs(diff);
                boundary_idx = (ranges[i + 1] < ranges[i]) ? i + 1 : i;
                disparity_diff = diff;
            }
        }

        is_curve = (max_disparity > disparity_threshold);

        // ---------- 목표 인덱스 ----------
        if (is_curve && boundary_idx != -1) {
            // 곡선 (Disparity 사용)
            float distance = std::min(ranges[boundary_idx], ranges[std::max(0, boundary_idx - 1)]);
            if (distance < 0.1) distance = 0.1;

            float offset_angle = atan((car_half_width + margin) / distance);
            float boundary_angle = msg->angle_min + boundary_idx * msg->angle_increment;

            float goal_angle = (disparity_diff < 0) ? 
                boundary_angle - offset_angle : boundary_angle + offset_angle;

            goal_idx = (int)((goal_angle - msg->angle_min) / msg->angle_increment);
            if (goal_idx < 0) goal_idx = 0;
            if (goal_idx >= n) goal_idx = n - 1;

        } else {
            // 직선 (가장 먼 포인트 중앙 추종)
            std::vector<int> max_indices;
            for (int i = 0; i < n; i++) {
                if (ranges[i] >= max_range) {
                    max_indices.push_back(i);
                }
            }
            if (!max_indices.empty()) {
                goal_idx = (max_indices.front() + max_indices.back()) / 2;
            }
        }

        // ---------- fallback 로직 ----------
        int valid_count = std::count_if(ranges.begin(), ranges.end(), [](float r){ return r > 0.0; });
        if (valid_count < 5 && goal_idx == -1) {
            // 유효 포인트가 거의 없으면 중앙 fallback
            goal_idx = n / 2;
            RCLCPP_WARN(this->get_logger(), "유효 포인트 부족 -> 중앙 fallback");
        }

        if (goal_idx == -1) {
            // 목표점 못찾으면 이전 목표 유지
            if (prev_goal_idx_ != -1) {
                goal_idx = prev_goal_idx_;
                RCLCPP_WARN(this->get_logger(), "목표점 못찾음 -> 이전 목표 유지");
            } else {
                // 이전 목표도 없다면 중앙으로 fallback
                goal_idx = n / 2;
                RCLCPP_WARN(this->get_logger(), "목표점 못찾음 -> 중앙 fallback");
            }
        }
        prev_goal_idx_ = goal_idx;

        // ---------- 목표 좌표 ----------
        float goal_angle = msg->angle_min + goal_idx * msg->angle_increment;
        float goal_dist = ranges[goal_idx];
        if (goal_dist <= 0.0) goal_dist = 1.0;

        float goal_x = goal_dist * cos(goal_angle);
        float goal_y = goal_dist * sin(goal_angle);

        // ---------- Pure Pursuit Lookahead ----------
        double Ld = std::sqrt(goal_x * goal_x + goal_y * goal_y);
        Ld = std::min(std::max(Ld + 0.3 * current_speed_, 0.5), 3.0);

        float steering_angle = std::atan2(2.0 * wheel_base * goal_y, (Ld * Ld));
        if (steering_angle < -0.34f) steering_angle = -0.34f;
        if (steering_angle > 0.34f) steering_angle = 0.34f;

        // 스무딩
        float alpha = 0.7;
        steering_angle = alpha * prev_steering_angle_ + (1 - alpha) * steering_angle;
        prev_steering_angle_ = steering_angle;

        // ---------- 속도 제어 ----------
        float angle_error = fabs(steering_angle);
        float max_speed = 4.0;
        float min_speed = 1.0; // 더 낮춰서 코너에서 멈추지 않도록 함
        float speed = max_speed - (angle_error / 0.34) * (max_speed - min_speed);
        if (speed < min_speed) speed = min_speed;
        if (speed > max_speed) speed = max_speed;

        // ---------- Ackermann 제어 명령 ----------
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "laser"; // frame_id 유지
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;
        drive_pub_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "[%s] steer=%.2f deg speed=%.2f Ld=%.2f",
                    is_curve ? "곡선" : "직선", steering_angle * 180.0 / M_PI, speed, Ld);
    }

    // ================= 멤버 변수 =================
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    float prev_steering_angle_;
    float current_speed_;
    int prev_goal_idx_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFollow>());
    rclcpp::shutdown();
    return 0;
}
