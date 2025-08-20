#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <cmath>
#include <vector>
#include <iostream>

class TrackCenterlineExtractor : public rclcpp::Node
{
public:
    TrackCenterlineExtractor() : Node("track_centerline_extractor")
    {
        // Parameters
        this->declare_parameter("map_pgm_file", "");
        this->declare_parameter("map_yaml_file", "");
        this->declare_parameter("output_dir", "");
        this->declare_parameter("subsample_period", 6);
        
        // Publishers
        centerline_pub_ = this->create_publisher<nav_msgs::msg::Path>("/centerline", 1);
        
        // Load map on startup
        loadAndProcessMap();
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerline_pub_;
    
    struct MapInfo {
        double resolution;
        std::vector<double> origin;
    };

    struct TrackPoint {
        double center_x, center_y;
        double left_x, right_x;
    };

    cv::Mat loadPGMFile(const std::string& filename)
    {
        cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        if(image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PGM file: %s", filename.c_str());
            throw std::runtime_error("Failed to load PGM file");
        }
        return image;
    }

    MapInfo loadYAMLFile(const std::string& filename)
    {
        std::ifstream fin(filename);
        if(!fin.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", filename.c_str());
            throw std::runtime_error("Failed to load YAML file");
        }

        MapInfo info;
        std::string line;
        while(std::getline(fin, line)) {
            if(line.find("resolution") != std::string::npos) {
                info.resolution = std::stod(line.substr(line.find(":") + 1));
            }
            else if(line.find("origin") != std::string::npos) {
                std::string values = line.substr(line.find("[") + 1, line.find("]") - line.find("[") - 1);
                std::stringstream ss(values);
                std::string value;
                while(std::getline(ss, value, ',')) {
                    info.origin.push_back(std::stod(value));
                }
            }
        }
        return info;
    }

    cv::Mat denoiseImage(const cv::Mat& input)
    {
        cv::Mat output;
        cv::medianBlur(input, output, 5);
        return output;
    }

    cv::Mat skeletonize(const cv::Mat& input)
    {
        cv::Mat binary;
        cv::threshold(input, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        
        cv::Mat skel = cv::Mat::zeros(input.size(), CV_8UC1);
        cv::Mat temp, eroded, dilated;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
        
        bool done;
        do {
            cv::erode(binary, eroded, element);
            cv::dilate(eroded, dilated, element);
            cv::subtract(binary, dilated, temp);
            cv::bitwise_or(skel, temp, skel);
            eroded.copyTo(binary);
            
            done = (cv::countNonZero(binary) == 0);
        } while (!done);
        
        return skel;
    }

    std::vector<cv::Point> findCenterlineCycle(const cv::Mat& skeleton)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(skeleton, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        
        if(contours.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No contours found in skeleton");
            return std::vector<cv::Point>();
        }
        
        // Find longest contour (assuming it's the centerline)
        auto longest = std::max_element(contours.begin(), contours.end(),
            [](const auto& a, const auto& b) { return a.size() < b.size(); });
            
        return *longest;
    }

    std::vector<cv::Point> subsamplePoints(const std::vector<cv::Point>& points, int period)
    {
        std::vector<cv::Point> subsampled;
        for(size_t i = 0; i < points.size(); i += period) {
            subsampled.push_back(points[i]);
        }
        return subsampled;
    }

    cv::Point2d getPerpendicularDirection(const std::vector<cv::Point>& centerline, int idx)
    {
        cv::Point2d direction;
        
        if(idx == 0) {
            direction = cv::Point2d(centerline[idx + 1] - centerline[idx]);
        } else if(idx == centerline.size() - 1) {
            direction = cv::Point2d(centerline[idx] - centerline[idx - 1]);
        } else {
            cv::Point2d dir1 = cv::Point2d(centerline[idx] - centerline[idx - 1]);
            cv::Point2d dir2 = cv::Point2d(centerline[idx + 1] - centerline[idx]);
            direction = (dir1 + dir2) * 0.5;
        }
        
        // Normalize
        double length = sqrt(direction.x * direction.x + direction.y * direction.y);
        if(length > 0) {
            direction.x /= length;
            direction.y /= length;
        }
        
        // Get perpendicular (90 degree rotation)
        return cv::Point2d(-direction.y, direction.x);
    }

    cv::Point2d findTrackEdge(const cv::Mat& map, cv::Point center, cv::Point2d direction, double max_distance = 5.0)
    {
        int max_pixels = static_cast<int>(max_distance / 0.05); // Assuming 0.05m resolution
        
        for(int distance = 1; distance < max_pixels; distance++) {
            int check_x = static_cast<int>(center.x + direction.x * distance);
            int check_y = static_cast<int>(center.y + direction.y * distance);
            
            // Check bounds
            if(check_x < 0 || check_x >= map.cols || check_y < 0 || check_y >= map.rows) {
                break;
            }
            
            // Check if we hit an obstacle (black pixel)
            if(map.at<uchar>(check_y, check_x) < 100) {
                return cv::Point2d(check_x, check_y);
            }
        }
        
        return cv::Point2d(-1, -1); // Not found
    }

    std::vector<TrackPoint> extractTrackEdges(const std::vector<cv::Point>& centerline, 
                                            const cv::Mat& map, const MapInfo& info)
    {
        std::vector<TrackPoint> track_points;
        
        for(size_t i = 0; i < centerline.size(); i++) {
            TrackPoint point;
            
            // Convert center point to world coordinates
            point.center_x = centerline[i].x * info.resolution + info.origin[0];
            point.center_y = centerline[i].y * info.resolution + info.origin[1];
            
            // Get perpendicular direction
            cv::Point2d perpendicular = getPerpendicularDirection(centerline, i);
            
            // Find left and right edges
            cv::Point2d left_edge = findTrackEdge(map, centerline[i], perpendicular);
            cv::Point2d right_edge = findTrackEdge(map, centerline[i], cv::Point2d(-perpendicular.x, -perpendicular.y));
            
            // Convert edges to world coordinates (only x coordinates needed)
            if(left_edge.x >= 0 && left_edge.y >= 0) {
                point.left_x = left_edge.x * info.resolution + info.origin[0];
            } else {
                point.left_x = 0.0;
            }
            
            if(right_edge.x >= 0 && right_edge.y >= 0) {
                point.right_x = right_edge.x * info.resolution + info.origin[0];
            } else {
                point.right_x = 0.0;
            }
            
            track_points.push_back(point);
        }
        
        return track_points;
    }

    void saveTrackPointsToCSV(const std::vector<TrackPoint>& track_points, const std::string& output_dir)
    {
        std::string filename = output_dir + "/track_edges.csv";
        std::ofstream file(filename);
        
        if(!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", filename.c_str());
            return;
        }
        
        // Write header
        file << "center_x,center_y,left_x,right_x\n";
        
        // Write data
        for(const auto& point : track_points) {
            file << point.center_x << "," << point.center_y << ","
                 << point.left_x << "," << point.right_x << "\n";
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "Track edges saved to: %s", filename.c_str());
    }

    void loadAndProcessMap()
    {
        try {
            // Load files
            std::string pgm_file = this->get_parameter("map_pgm_file").as_string();
            std::string yaml_file = this->get_parameter("map_yaml_file").as_string();
            std::string output_dir = this->get_parameter("output_dir").as_string();
            int subsample_period = this->get_parameter("subsample_period").as_int();

            cv::Mat image = loadPGMFile(pgm_file);
            MapInfo map_info = loadYAMLFile(yaml_file);

            // Process image
            cv::Mat denoised = denoiseImage(image);
            cv::Mat skeleton = skeletonize(denoised);
            std::vector<cv::Point> centerline = findCenterlineCycle(skeleton);
            std::vector<cv::Point> subsampled = subsamplePoints(centerline, subsample_period);

            // Extract track edges
            std::vector<TrackPoint> track_points = extractTrackEdges(subsampled, image, map_info);
            
            // Save to CSV
            saveTrackPointsToCSV(track_points, output_dir);

            // Convert to world coordinates and publish centerline
            publishCenterline(subsampled, map_info);
            
            RCLCPP_INFO(this->get_logger(), "Successfully processed map and extracted centerline with edges");
        }
        catch(const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing map: %s", e.what());
        }
    }

    void publishCenterline(const std::vector<cv::Point>& pixels, const MapInfo& info)
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "map";

        for(const auto& pixel : pixels) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            
            // Convert pixel coordinates to world coordinates
            pose.pose.position.x = pixel.x * info.resolution + info.origin[0];
            pose.pose.position.y = pixel.y * info.resolution + info.origin[1];
            pose.pose.position.z = 0.0;
            
            // Set orientation (identity quaternion)
            pose.pose.orientation.w = 1.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;

            path.poses.push_back(pose);
        }

        centerline_pub_->publish(path);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackCenterlineExtractor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}