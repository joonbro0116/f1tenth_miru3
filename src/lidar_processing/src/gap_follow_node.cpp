#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class GapFollow : public rclcpp::Node {
public:
    GapFollow()
    : Node("gap_follow_node")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&GapFollow::lidar_callback, this, std::placeholders::_1)
        );
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    void preprocess_lidar(float* ranges, int num_ranges) {
        const int window_size = 5;
        const float max_range = 2.0f;  // 관찰할 최대 거리 2m로 제한
        std::vector<float> temp(num_ranges);
        for (int i = 0; i < num_ranges; ++i) {
            float sum = 0.0f;
            int count = 0;
            for (int j = -window_size/2; j <= window_size/2; ++j) {
                int idx = i + j;
                if (idx >= 0 && idx < num_ranges) {
                    // 2m 초과 거리는 무시
                    float r = (ranges[idx] > max_range ? max_range : ranges[idx]);
                    sum += r;
                    ++count;
                }
            }
            temp[i] = (count > 0 ? sum / count : 0.0f);
        }
        for (int i = 0; i < num_ranges; ++i) {
            // 클램핑 기능 유지
            ranges[i] = (temp[i] > max_range ? max_range : temp[i]);
        }
    }

    void find_max_gap(const std::vector<float>& ranges, int indice[2]) {
        const float free_threshold = 0.001f;
        int curr_start = -1, curr_length = 0;
        int best_start = 0, best_end = 0, best_length = 0;
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > free_threshold) {
                if (curr_start < 0) {
                    curr_start = static_cast<int>(i);
                    curr_length = 0;
                }
                ++curr_length;
                if (curr_length > best_length) {
                    best_length = curr_length;
                    best_start  = curr_start;
                    best_end    = static_cast<int>(i);
                }
            } else {
                curr_start  = -1;
                curr_length = 0;
            }
        }
        indice[0] = best_start;
        indice[1] = best_end;
    }

    int find_best_point(const int indice[2]) {
        return (indice[0] + indice[1]) / 2;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        if (scan_msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty scan received");
            return;
        }
        int num_ranges = static_cast<int>(scan_msg->ranges.size());
        std::vector<float> proc_ranges = scan_msg->ranges;

        // ROI: 전방 ±90°만 사용
        double angle_min = scan_msg->angle_min;
        double angle_inc = scan_msg->angle_increment;
        double pi = std::acos(-1.0);
        int idx_start = std::max(0, static_cast<int>(std::ceil(( -pi/2 - angle_min) / angle_inc)));
        int idx_end   = std::min(num_ranges-1, static_cast<int>(std::floor(( pi/2 - angle_min) / angle_inc)));
        for (int i = 0; i < idx_start; ++i) proc_ranges[i] = 0.0f;
        for (int i = idx_end+1; i < num_ranges; ++i) proc_ranges[i] = 0.0f;

        // 1) 전처리 (2m 거리 범위로 클램핑 및 스무딩)
        preprocess_lidar(proc_ranges.data(), num_ranges);
        
        // 2) bubble 생성
        float min_dist = std::numeric_limits<float>::infinity();
        int min_idx = idx_start;
        for (int i = idx_start; i <= idx_end; ++i) {
            if (proc_ranges[i] < min_dist) {
                min_dist = proc_ranges[i];
                min_idx = i;
            }
        }
        const int bubble_radius = 30;
        int start_b = std::max(idx_start, min_idx - bubble_radius);
        int end_b   = std::min(idx_end, min_idx + bubble_radius);
        for (int i = start_b; i <= end_b; ++i) proc_ranges[i] = 0.0f;

        // 3) max gap 찾기
        int best_indices[2] = {0, 0};
        find_max_gap(proc_ranges, best_indices);
        // 4) best point 찾기
        int best_point_idx = find_best_point(best_indices);

        // 5) 조향 각도 = 차량 중심 인덱스 대비 offset * angle_inc
        int center_idx = num_ranges / 2;
        int offset_idx = best_point_idx - center_idx;
        float steering_angle = offset_idx * angle_inc;

        RCLCPP_INFO(this->get_logger(),
            "center=%d, best=%d, offset=%d, steer=%.3f rad, max_range=2.0m",
            center_idx, best_point_idx, offset_idx, steering_angle);

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header = scan_msg->header;
        drive_msg.drive.speed          = 1.0f;
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
