#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

class GapFollow : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    GapFollow() : Node("gap_follow_node")
    {
        // Create ROS subscribers and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&GapFollow::lidar_callback, this, std::placeholders::_1));
        // Publisher for AckermannDriveStamped messages
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    void preprocess_lidar(float* ranges)
    {   
        const int num_ranges = 1080; // LiDAR 데이터 포인트 개수 (센서에 따라 다름)
        const int window_size = 5;   // 이동 평균 윈도우 크기 (홀수 권장)
        const float max_range = 3.0; // 최대 허용 거리

        float temp[num_ranges];

        // 이동 평균 필터 적용
        for (int i = 0; i < num_ranges; ++i) {
            float sum = 0.0;
            int count = 0;
            for (int j = -window_size/2; j <= window_size/2; ++j) {
                int idx = i + j;
                if (idx >= 0 && idx < num_ranges) {
                    sum += ranges[idx];
                    count++;
                }
            }
            temp[i] = sum / count;
        }

        // 최대 거리 제한 적용
        for (int i = 0; i < num_ranges; ++i) {
            if (temp[i] > max_range) {
                ranges[i] = max_range;
            } else {
                ranges[i] = temp[i];
            }
        }
        return;
    }

    void find_max_gap(const std::vector<float>& ranges, int* indice)
    {   
        int max_start = 0, max_end = 0, max_len = 0;
        int curr_start = -1, curr_len = 0;

        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > 0.001) { // free space (0이면 장애물 또는 bubble)
                if (curr_start == -1) curr_start = i;
                curr_len++;
                if (curr_len > max_len) {
                    max_len = curr_len;
                    max_start = curr_start;
                    max_end = i;
                }
            } else {
                curr_start = -1;
                curr_len = 0;
            }
        }
        indice[0] = max_start;
        indice[1] = max_end;
    }

    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 

        // Find the best point in the gap 

        // Publish Drive message
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFollow>());
    rclcpp::shutdown();
    return 0;
}