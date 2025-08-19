#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <memory>
#include <limits>
#include <queue>
#include <fstream>

class MinCurvaturePlanner : public rclcpp::Node
{
public:
    MinCurvaturePlanner() : Node("min_curvature_planner")
    {
        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        racing_line_pub_ = this->create_publisher<nav_msgs::msg::Path>("/racing_line", 10);
        boundary_pub_ = this->create_publisher<nav_msgs::msg::Path>("/track_boundary", 10);

        // Parameters
        this->declare_parameter("map_pgm_file", "");
        this->declare_parameter("map_resolution", 0.05);
        this->declare_parameter("map_origin_x", 0.0);
        this->declare_parameter("map_origin_y", 0.0);
        this->declare_parameter("track_width", 2.0);
        this->declare_parameter("safety_margin", 0.3);
        this->declare_parameter("num_waypoints", 100);
        this->declare_parameter("optimization_weight", 1.0);
        this->declare_parameter("min_radius", 0.5);
        this->declare_parameter("distance_threshold", 2.0);
        this->declare_parameter("boundary_erosion", 3);

        std::string pgm_file = this->get_parameter("map_pgm_file").as_string();
        double map_resolution = this->get_parameter("map_resolution").as_double();
        double map_origin_x = this->get_parameter("map_origin_x").as_double();
        double map_origin_y = this->get_parameter("map_origin_y").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        safety_margin_ = this->get_parameter("safety_margin").as_double();
        num_waypoints_ = this->get_parameter("num_waypoints").as_int();
        opt_weight_ = this->get_parameter("optimization_weight").as_double();
        min_radius_ = this->get_parameter("min_radius").as_double();
        distance_threshold_ = this->get_parameter("distance_threshold").as_double();
        boundary_erosion_ = this->get_parameter("boundary_erosion").as_int();

        // Load PGM as raw pixel values
        int width, height;
        loadMapFromPGM(pgm_file, map_pixels_, width, height);

        map_width_ = width;
        map_height_ = height;
        map_resolution_ = map_resolution;
        map_origin_x_ = map_origin_x;
        map_origin_y_ = map_origin_y;

        RCLCPP_INFO(this->get_logger(), "PGM 맵 로드 완료: %dx%d", width, height);

        // 트랙 바운더리 및 중심선 추출
        if (detectTrackBoundary()) {
            createTrackInteriorMask();
            extractTrackCenterline();
        }
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

    // ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr racing_line_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr boundary_pub_;

    // Parameters
    double track_width_;
    double safety_margin_;
    int num_waypoints_;
    double opt_weight_;
    double min_radius_;
    double distance_threshold_;
    int boundary_erosion_;

    // Map data (raw pixel values)
    std::vector<uint8_t> map_pixels_;
    int map_width_;
    int map_height_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;

    std::vector<Point2D> track_centerline_;
    std::vector<Point2D> track_boundary_;
    std::vector<std::vector<bool>> track_interior_mask_;

    void loadMapFromPGM(const std::string& pgm_path, std::vector<uint8_t>& data, int& width, int& height)
    {
        cv::Mat img = cv::imread(pgm_path, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            throw std::runtime_error("PGM 파일을 읽을 수 없습니다: " + pgm_path);
        }
        width = img.cols;
        height = img.rows;
        data.resize(width * height);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                data[y * width + x] = img.at<uint8_t>(y, x);
            }
        }
    }

    // 자유공간(254)만 true로 하는 free_space 생성
    std::vector<std::vector<bool>> getFreeSpaceMask() {
        std::vector<std::vector<bool>> free_space(map_height_, std::vector<bool>(map_width_, false));
        int free_count = 0;
        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                uint8_t val = map_pixels_[y * map_width_ + x];
                if (val == 254) {
                    free_space[y][x] = true;
                    free_count++;
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "자유공간(254) 셀: %d (%.1f%%)", free_count,
            (static_cast<double>(free_count) / (map_width_ * map_height_)) * 100.0);
        return free_space;
    }

    // 외곽 폐곡선 기반 트랙 영역 추출
    std::vector<std::vector<bool>> findTrackRegionWithinOuterBoundary()
    {
        auto free_space = getFreeSpaceMask();

        // 1. 외곽 폐곡선 찾기 (윤곽선 추출)
        std::vector<GridPoint> outer_boundary = findOuterBoundaryContour(free_space, map_width_, map_height_);

        if (outer_boundary.empty()) {
            RCLCPP_WARN(this->get_logger(), "외곽 폐곡선을 찾을 수 없습니다.");
            return findLargestConnectedRegion(free_space, map_width_, map_height_); // fallback
        }

        // 2. 외곽 폐곡선 내부 영역만 마스킹
        std::vector<std::vector<bool>> interior_mask = createInteriorMask(outer_boundary, map_width_, map_height_);

        // 3. 자유공간과 내부 영역의 교집합
        std::vector<std::vector<bool>> valid_region(map_height_, std::vector<bool>(map_width_, false));
        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                valid_region[y][x] = free_space[y][x] && interior_mask[y][x];
            }
        }

        RCLCPP_INFO(this->get_logger(), "외곽 폐곡선 기반 트랙 영역 추출 완료");
        return valid_region;
    }

    // 가장 바깥쪽 윤곽선(contour)을 찾는 함수
    std::vector<GridPoint> findOuterBoundaryContour(const std::vector<std::vector<bool>>& free_space, int width, int height)
    {
        // 1. 모든 연결된 컴포넌트 찾기
        std::vector<std::vector<int>> component_map(height, std::vector<int>(width, -1));
        std::vector<std::vector<GridPoint>> all_contours;
        int component_id = 0;
        std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (free_space[y][x] && !visited[y][x]) {
                    std::vector<GridPoint> component_points;
                    floodFillAndMarkComponent(free_space, visited, component_map, x, y, component_id, component_points, width, height);
                    if (component_points.size() > 100) {
                        std::vector<GridPoint> contour = extractContour(component_points, free_space, width, height);
                        if (!contour.empty()) {
                            all_contours.push_back(contour);
                        }
                    }
                    component_id++;
                }
            }
        }
        if (all_contours.empty()) return {};
        std::vector<GridPoint> outer_contour;
        double max_area = 0;
        for (const auto& contour : all_contours) {
            double area = calculateContourBoundingArea(contour);
            if (area > max_area) {
                max_area = area;
                outer_contour = contour;
            }
        }
        RCLCPP_INFO(this->get_logger(), "외곽 윤곽선 찾기 완료: %zu개 포인트, 면적: %.2f", outer_contour.size(), max_area);
        return outer_contour;
    }

    // Flood fill로 컴포넌트 마킹 및 포인트 수집
    void floodFillAndMarkComponent(const std::vector<std::vector<bool>>& free_space,
                                  std::vector<std::vector<bool>>& visited,
                                  std::vector<std::vector<int>>& component_map,
                                  int start_x, int start_y, int component_id,
                                  std::vector<GridPoint>& component_points,
                                  int width, int height)
    {
        std::queue<GridPoint> queue;
        queue.push(GridPoint(start_x, start_y));
        visited[start_y][start_x] = true;
        component_map[start_y][start_x] = component_id;
        int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        while (!queue.empty()) {
            GridPoint current = queue.front();
            queue.pop();
            component_points.push_back(current);
            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];
                if (nx >= 0 && nx < width && ny >= 0 && ny < height &&
                    free_space[ny][nx] && !visited[ny][nx]) {
                    queue.push(GridPoint(nx, ny));
                    visited[ny][nx] = true;
                    component_map[ny][nx] = component_id;
                }
            }
        }
    }

    // 컴포넌트의 윤곽선 추출 (Moore boundary tracing 알고리즘 사용)
    std::vector<GridPoint> extractContour(const std::vector<GridPoint>& component_points,
                                         const std::vector<std::vector<bool>>& free_space,
                                         int width, int height)
    {
        if (component_points.empty()) return {};
        // 가장 왼쪽 위 점을 시작점으로 찾기
        GridPoint start_point = component_points[0];
        for (const auto& point : component_points) {
            if (point.y < start_point.y || (point.y == start_point.y && point.x < start_point.x)) {
                start_point = point;
            }
        }
        // Moore boundary tracing
        std::vector<GridPoint> contour;
        GridPoint current = start_point;
        int direction = 0; // 시작 방향 (북쪽부터)
        // 8방향: N, NE, E, SE, S, SW, W, NW
        int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
        int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};
        do {
            contour.push_back(current);
            // 현재 방향에서 반시계방향으로 시작해서 경계 찾기
            int search_dir = (direction + 6) % 8; // 반시계방향 90도
            bool found = false;
            for (int i = 0; i < 8; ++i) {
                int check_dir = (search_dir + i) % 8;
                int nx = current.x + dx[check_dir];
                int ny = current.y + dy[check_dir];
                if (nx >= 0 && nx < width && ny >= 0 && ny < height && free_space[ny][nx]) {
                    current = GridPoint(nx, ny);
                    direction = check_dir;
                    found = true;
                    break;
                }
            }
            if (!found) break;
        } while (!(current == start_point) && contour.size() < 10000); // 무한루프 방지
        return contour;
    }

    // 윤곽선의 bounding box 면적 계산
    double calculateContourBoundingArea(const std::vector<GridPoint>& contour)
    {
        if (contour.empty()) return 0;
        int min_x = contour[0].x, max_x = contour[0].x;
        int min_y = contour[0].y, max_y = contour[0].y;
        for (const auto& point : contour) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
        return static_cast<double>((max_x - min_x) * (max_y - min_y));
    }

    // 폐곡선 내부 영역 마스크 생성 (ray casting 알고리즘)
    std::vector<std::vector<bool>> createInteriorMask(const std::vector<GridPoint>& boundary, int width, int height)
    {
        std::vector<std::vector<bool>> interior_mask(height, std::vector<bool>(width, false));
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (isPointInsidePolygon(x, y, boundary)) {
                    interior_mask[y][x] = true;
                }
            }
        }
        return interior_mask;
    }

    // Ray casting을 이용한 점-폴리곤 내부 판정
    bool isPointInsidePolygon(int x, int y, const std::vector<GridPoint>& polygon)
    {
        if (polygon.size() < 3) return false;
        int intersections = 0;
        int n = polygon.size();
        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            int x1 = polygon[i].x, y1 = polygon[i].y;
            int x2 = polygon[j].x, y2 = polygon[j].y;
            // 수평 레이와 폴리곤 엣지의 교차점 확인
            if (((y1 > y) != (y2 > y)) &&
                (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1)) {
                intersections++;
            }
        }
        return (intersections % 2) == 1; // 홀수면 내부
    }

    std::vector<std::vector<bool>> findLargestConnectedRegion(const std::vector<std::vector<bool>>& free_space, int width, int height)
    {
        std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
        std::vector<std::vector<bool>> largest_region(height, std::vector<bool>(width, false));
        int max_region_size = 0;
        int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (free_space[y][x] && !visited[y][x]) {
                    std::queue<GridPoint> queue;
                    std::vector<GridPoint> current_region;
                    queue.push(GridPoint(x, y));
                    visited[y][x] = true;
                    while (!queue.empty()) {
                        GridPoint current = queue.front();
                        queue.pop();
                        current_region.push_back(current);
                        for (int i = 0; i < 8; ++i) {
                            int nx = current.x + dx[i];
                            int ny = current.y + dy[i];
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height &&
                                free_space[ny][nx] && !visited[ny][nx]) {
                                queue.push(GridPoint(nx, ny));
                                visited[ny][nx] = true;
                            }
                        }
                    }
                    if (static_cast<int>(current_region.size()) > max_region_size) {
                        max_region_size = current_region.size();
                        std::fill(largest_region.begin(), largest_region.end(), std::vector<bool>(width, false));
                        for (const auto& point : current_region) {
                            largest_region[point.y][point.x] = true;
                        }
                    }
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "가장 큰 연결 영역 크기: %d개 셀", max_region_size);
        return largest_region;
    }

    std::vector<GridPoint> extractBoundaryPoints(const std::vector<std::vector<bool>>& region, int width, int height)
    {
        std::vector<GridPoint> boundary_points;
        int dx[] = {-1, 0, 1, 0};
        int dy[] = {0, -1, 0, 1};
        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                if (region[y][x]) {
                    bool is_boundary = false;
                    for (int i = 0; i < 4; ++i) {
                        int nx = x + dx[i];
                        int ny = y + dy[i];
                        if (!region[ny][nx]) {
                            is_boundary = true;
                            break;
                        }
                    }
                    if (is_boundary) {
                        boundary_points.emplace_back(x, y);
                    }
                }
            }
        }
        return boundary_points;
    }

    void applyErosion(std::vector<std::vector<bool>>& mask, int erosion_size, int width, int height)
    {
        std::vector<std::vector<bool>> eroded = mask;
        for (int iter = 0; iter < erosion_size; ++iter) {
            std::vector<std::vector<bool>> temp = eroded;
            for (int y = 1; y < height - 1; ++y) {
                for (int x = 1; x < width - 1; ++x) {
                    if (eroded[y][x]) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dx = -1; dx <= 1; ++dx) {
                                if (!eroded[y + dy][x + dx]) {
                                    temp[y][x] = false;
                                    goto next_point;
                                }
                            }
                        }
                        next_point:;
                    }
                }
            }
            eroded = temp;
        }
        mask = eroded;
    }

    // 트랙 바운더리 검출
    bool detectTrackBoundary()
    {
        RCLCPP_INFO(this->get_logger(), "외곽 폐곡선 기반 트랙 바운더리 검출 시작...");
        std::vector<std::vector<bool>> valid_track_region = findTrackRegionWithinOuterBoundary();
        if (valid_track_region.empty()) {
            RCLCPP_WARN(this->get_logger(), "유효한 트랙 영역을 찾을 수 없습니다.");
            return false;
        }
        std::vector<GridPoint> boundary_points = extractBoundaryPoints(valid_track_region, map_width_, map_height_);
        if (boundary_points.size() < 10) {
            RCLCPP_WARN(this->get_logger(), "충분한 트랙 바운더리 포인트를 찾을 수 없습니다.");
            return false;
        }
        track_boundary_.clear();
        for (const auto& point : boundary_points) {
            double world_x = map_origin_x_ + point.x * map_resolution_;
            double world_y = map_origin_y_ + (map_height_ - 1 - point.y) * map_resolution_;
            track_boundary_.emplace_back(world_x, world_y);
        }
        RCLCPP_INFO(this->get_logger(), "외곽 폐곡선 기반 트랙 바운더리 검출 완료: %zu개 포인트", track_boundary_.size());
        publishBoundary();
        return true;
    }

    void createTrackInteriorMask()
    {
        track_interior_mask_ = findTrackRegionWithinOuterBoundary();
        if (boundary_erosion_ > 0) {
            applyErosion(track_interior_mask_, boundary_erosion_, map_width_, map_height_);
        }
        RCLCPP_INFO(this->get_logger(), "외곽 폐곡선 기반 트랙 내부 마스크 생성 완료");
    }

    void extractTrackCenterline()
    {
        if (track_interior_mask_.empty()) return;
        RCLCPP_INFO(this->get_logger(), "트랙 내부에서 중심선 추출 시작...");
        std::vector<std::vector<double>> distance_map(map_height_, std::vector<double>(map_width_, 0.0));
        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                if (track_interior_mask_[y][x]) {
                    double min_dist = findMinDistanceToTrackBoundary(x, y, map_width_, map_height_);
                    distance_map[y][x] = min_dist;
                }
            }
        }
        std::vector<Point2D> candidate_points;
        for (int y = 1; y < map_height_ - 1; ++y) {
            for (int x = 1; x < map_width_ - 1; ++x) {
                if (track_interior_mask_[y][x] && isLocalMaximum(x, y, distance_map)) {
                    double world_x = map_origin_x_ + x * map_resolution_;
                    double world_y = map_origin_y_ + (map_height_ - 1 - y) * map_resolution_;
                    candidate_points.emplace_back(world_x, world_y);
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "트랙 내부 중심선 후보: %zu개", candidate_points.size());
        if (candidate_points.size() > 10) {
            track_centerline_ = sortAndConnectCenterlinePoints(candidate_points);
            RCLCPP_INFO(this->get_logger(), "트랙 내부 중심선 추출 완료: %zu개 포인트", track_centerline_.size());
            publishCenterlineAsPath();
        } else {
            RCLCPP_WARN(this->get_logger(), "충분한 중심선 포인트를 찾을 수 없습니다.");
        }
    }

    double findMinDistanceToTrackBoundary(int x, int y, int width, int height)
    {
        double min_dist = std::numeric_limits<double>::max();
        int search_radius = 20;
        for (int dy = -search_radius; dy <= search_radius; ++dy) {
            for (int dx = -search_radius; dx <= search_radius; ++dx) {
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    if (!track_interior_mask_[ny][nx]) {
                        double dist = std::sqrt(dx*dx + dy*dy);
                        min_dist = std::min(min_dist, dist);
                    }
                }
            }
        }
        return min_dist;
    }

    bool isLocalMaximum(int x, int y, const std::vector<std::vector<double>>& distance_map)
    {
        double center_val = distance_map[y][x];
        if (center_val < 1.0) return false; // distance_threshold_ 대신 고정값 사용
        int radius = 1; // 반경 감소로 더 많은 후보점 생성
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx == 0 && dy == 0) continue;
                int ny = y + dy;
                int nx = x + dx;
                if (ny >= 0 && ny < static_cast<int>(distance_map.size()) &&
                    nx >= 0 && nx < static_cast<int>(distance_map[0].size())) {
                    if (track_interior_mask_[ny][nx] && distance_map[ny][nx] > center_val + 0.1) { // 더 관대한 조건
                        return false;
                    }
                }
            }
        }
        return true;
    }

    void publishBoundary()
    {
        if (track_boundary_.empty()) return;
        nav_msgs::msg::Path boundary_msg;
        boundary_msg.header.stamp = this->get_clock()->now();
        boundary_msg.header.frame_id = "map";
        for (const auto& point : track_boundary_) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = boundary_msg.header;
            pose_stamped.pose.position.x = point.x;
            pose_stamped.pose.position.y = point.y;
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;
            boundary_msg.poses.push_back(pose_stamped);
        }
        boundary_pub_->publish(boundary_msg);
        RCLCPP_INFO(this->get_logger(), "트랙 바운더리 퍼블리시 완료: %zu개 포인트", track_boundary_.size());
    }

    std::vector<Point2D> sortAndConnectCenterlinePoints(const std::vector<Point2D>& candidate_points)
    {
        if (candidate_points.empty()) return {};
        std::vector<Point2D> sorted_points;
        std::vector<bool> used(candidate_points.size(), false);
        
        // 가장 왼쪽 점을 시작점으로 선택
        int start_idx = 0;
        for (size_t i = 1; i < candidate_points.size(); ++i) {
            if (candidate_points[i].x < candidate_points[start_idx].x) {
                start_idx = i;
            }
        }
        
        sorted_points.push_back(candidate_points[start_idx]);
        used[start_idx] = true;
        
        // 가장 가까운 점들을 순차적으로 연결
        while (sorted_points.size() < candidate_points.size()) {
            Point2D current = sorted_points.back();
            double min_dist = std::numeric_limits<double>::max();
            int next_idx = -1;
            
            for (size_t i = 0; i < candidate_points.size(); ++i) {
                if (!used[i]) {
                    double dist = (candidate_points[i] - current).norm();
                    if (dist < min_dist && dist < 5.0) { // 거리 임계값 증가
                        min_dist = dist;
                        next_idx = i;
                    }
                }
            }
            
            if (next_idx == -1) break;
            sorted_points.push_back(candidate_points[next_idx]);
            used[next_idx] = true;
        }
        
        // Closed loop를 위해 시작점과 끝점 연결 확인
        if (sorted_points.size() >= 3) {
            Point2D first_point = sorted_points[0];
            Point2D last_point = sorted_points.back();
            double loop_distance = (last_point - first_point).norm();
            
            // 시작점과 끝점이 충분히 가까우면 closed loop로 처리
            if (loop_distance < 5.0) {
                // 마지막 점과 시작점 사이에 중간점 추가하여 부드러운 연결
                Point2D mid_point = (last_point + first_point) * 0.5;
                sorted_points.push_back(mid_point);
                sorted_points.push_back(first_point); // 시작점으로 다시 연결
                RCLCPP_INFO(this->get_logger(), "Closed loop 중심선 생성 완료 (거리: %.2fm)", loop_distance);
            } else {
                RCLCPP_WARN(this->get_logger(), "Closed loop 연결 실패 - 시작점과 끝점 거리: %.2fm", loop_distance);
            }
        }
        
        return smoothCenterline(sorted_points);
    }

    std::vector<Point2D> smoothCenterline(const std::vector<Point2D>& raw_centerline)
    {
        if (raw_centerline.size() < 3) return raw_centerline;
        std::vector<Point2D> smoothed = raw_centerline;
        for (int iter = 0; iter < 3; ++iter) {
            for (size_t i = 1; i < smoothed.size() - 1; ++i) {
                Point2D prev = smoothed[i-1];
                Point2D curr = smoothed[i];
                Point2D next = smoothed[i+1];
                Point2D new_point = curr * 0.6 + prev * 0.2 + next * 0.2;
                smoothed[i] = new_point;
            }
        }
        return smoothed;
    }

    void publishCenterlineAsPath()
    {
        if (track_centerline_.empty()) return;
        nav_msgs::msg::Path centerline_msg;
        centerline_msg.header.stamp = this->get_clock()->now();
        centerline_msg.header.frame_id = "map";
        for (const auto& point : track_centerline_) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = centerline_msg.header;
            pose_stamped.pose.position.x = point.x;
            pose_stamped.pose.position.y = point.y;
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;
            centerline_msg.poses.push_back(pose_stamped);
        }
        racing_line_pub_->publish(centerline_msg);
        saveCenterlineToCSV();
        RCLCPP_INFO(this->get_logger(), "중심선 퍼블리시 완료: %zu개 포인트", track_centerline_.size());
    }

    std::vector<Point2D> calculateEdgePoints(const std::vector<Point2D>& centerline, double half_width, bool is_left)
    {
        std::vector<Point2D> edge_points;
        if (centerline.size() < 2) return edge_points;
        
        for (size_t i = 0; i < centerline.size(); ++i) {
            Point2D current = centerline[i];
            Point2D direction;
            
            // Calculate direction vector using neighboring points
            if (i == 0) {
                // First point: use direction to next point
                Point2D prev = centerline.back(); // Last point (closed loop)
                Point2D next = centerline[i + 1];
                Point2D dir1 = current - prev;
                Point2D dir2 = next - current;
                direction = (dir1 + dir2) * 0.5;
            } else if (i == centerline.size() - 1) {
                // Last point: use direction from previous point
                Point2D prev = centerline[i - 1];
                Point2D next = centerline[0]; // First point (closed loop)
                Point2D dir1 = current - prev;
                Point2D dir2 = next - current;
                direction = (dir1 + dir2) * 0.5;
            } else {
                // Middle points: average of incoming and outgoing directions
                Point2D prev = centerline[i - 1];
                Point2D next = centerline[i + 1];
                Point2D dir1 = current - prev;
                Point2D dir2 = next - current;
                direction = (dir1 + dir2) * 0.5;
            }
            
            // Normalize direction vector
            double norm = direction.norm();
            if (norm > 0) {
                direction = direction * (1.0 / norm);
            }
            
            // Calculate perpendicular vector
            Point2D perpendicular;
            if (is_left) {
                perpendicular = Point2D(-direction.y, direction.x); // Rotate 90 degrees counter-clockwise
            } else {
                perpendicular = Point2D(direction.y, -direction.x); // Rotate 90 degrees clockwise
            }
            
            // Calculate edge point
            Point2D edge_point = current + perpendicular * half_width;
            edge_points.push_back(edge_point);
        }
        
        return edge_points;
    }
    
    void saveCenterlineToCSV()
    {
        if (track_centerline_.empty()) return;
        
        // Calculate left and right distances to track boundaries for each centerline point
        std::vector<double> left_distances, right_distances;
        calculateLeftRightBoundaryDistances(left_distances, right_distances);
        
        // Save centerline with left/right distances in single CSV file
        std::string track_data_path = "/home/f1/f1tenth_ws/maps/centerline_with_distances.csv";
        std::ofstream track_file(track_data_path);
        
        if (!track_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "CSV 파일을 생성할 수 없습니다: %s", track_data_path.c_str());
            return;
        }
        
        // Write header
        track_file << "x,y,left_distance,right_distance\n";
        
        // Write centerline points with distances
        for (size_t i = 0; i < track_centerline_.size(); ++i) {
            const auto& point = track_centerline_[i];
            double left_dist = (i < left_distances.size()) ? left_distances[i] : 0.0;
            double right_dist = (i < right_distances.size()) ? right_distances[i] : 0.0;
            
            track_file << point.x << "," << point.y << "," << left_dist << "," << right_dist << "\n";
        }
        
        track_file.close();
        RCLCPP_INFO(this->get_logger(), "중심선과 양쪽 거리가 CSV로 저장되었습니다: %s", track_data_path.c_str());
        
        // Also save the original centerline.csv for compatibility
        std::string centerline_path = "/home/f1/f1tenth_ws/maps/centerline.csv";
        std::ofstream centerline_file(centerline_path);
        
        if (centerline_file.is_open()) {
            centerline_file << "x,y\n";
            for (const auto& point : track_centerline_) {
                centerline_file << point.x << "," << point.y << "\n";
            }
            centerline_file.close();
            RCLCPP_INFO(this->get_logger(), "중심선이 CSV로 저장되었습니다: %s", centerline_path.c_str());
        }
        
        // Calculate average distances for logging
        if (!left_distances.empty() && !right_distances.empty()) {
            double avg_left = std::accumulate(left_distances.begin(), left_distances.end(), 0.0) / left_distances.size();
            double avg_right = std::accumulate(right_distances.begin(), right_distances.end(), 0.0) / right_distances.size();
            RCLCPP_INFO(this->get_logger(), "평균 거리 - 왼쪽: %.2fm, 오른쪽: %.2fm", avg_left, avg_right);
        }
    }
    
    void calculateLeftRightBoundaryDistances(std::vector<double>& left_distances, std::vector<double>& right_distances)
    {
        left_distances.clear();
        right_distances.clear();
        
        // Create distance transform map
        cv::Mat distance_map = createDistanceTransformMap();
        
        for (size_t i = 0; i < track_centerline_.size(); ++i) {
            const auto& point = track_centerline_[i];
            
            // Convert world coordinates to grid coordinates
            int grid_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_);
            int grid_y = static_cast<int>((map_height_ - 1) - (point.y - map_origin_y_) / map_resolution_);
            
            // Calculate direction vector
            Point2D direction;
            if (i == 0) {
                Point2D prev = track_centerline_.back();
                Point2D next = track_centerline_[i + 1];
                Point2D dir1 = point - prev;
                Point2D dir2 = next - point;
                direction = (dir1 + dir2) * 0.5;
            } else if (i == track_centerline_.size() - 1) {
                Point2D prev = track_centerline_[i - 1];
                Point2D next = track_centerline_[0];
                Point2D dir1 = point - prev;
                Point2D dir2 = next - point;
                direction = (dir1 + dir2) * 0.5;
            } else {
                Point2D prev = track_centerline_[i - 1];
                Point2D next = track_centerline_[i + 1];
                Point2D dir1 = point - prev;
                Point2D dir2 = next - point;
                direction = (dir1 + dir2) * 0.5;
            }
            
            // Normalize direction vector
            double norm = direction.norm();
            if (norm > 0) {
                direction = direction * (1.0 / norm);
            }
            
            // Calculate perpendicular vectors (left and right)
            Point2D left_perp(-direction.y, direction.x);   // 90° counter-clockwise
            Point2D right_perp(direction.y, -direction.x);  // 90° clockwise
            
            // Find distances to boundaries in left and right directions using distance transform
            double left_dist = findDistanceInDirectionUsingTransform(grid_x, grid_y, left_perp, distance_map);
            double right_dist = findDistanceInDirectionUsingTransform(grid_x, grid_y, right_perp, distance_map);
            
            left_distances.push_back(left_dist);
            right_distances.push_back(right_dist);
        }
        
        RCLCPP_INFO(this->get_logger(), "Distance Transform으로 왼쪽/오른쪽 경계 거리 계산 완료: %zu개 포인트", left_distances.size());
    }
    
    cv::Mat createDistanceTransformMap()
    {
        // Create binary map (255 = free space, 0 = obstacle)
        cv::Mat binary_map(map_height_, map_width_, CV_8UC1);
        
        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                uint8_t pixel_value = map_pixels_[y * map_width_ + x];
                binary_map.at<uint8_t>(y, x) = (pixel_value == 254) ? 255 : 0;
            }
        }
        
        // Calculate distance transform
        cv::Mat distance_map;
        cv::distanceTransform(binary_map, distance_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        
        RCLCPP_INFO(this->get_logger(), "Distance Transform 맵 생성 완료: %dx%d", map_width_, map_height_);
        return distance_map;
    }
    
    double findDistanceInDirectionUsingTransform(int start_x, int start_y, const Point2D& direction, const cv::Mat& distance_map)
    {
        // Check bounds
        if (start_x < 0 || start_x >= map_width_ || start_y < 0 || start_y >= map_height_) {
            return 0.0;
        }
        
        // Sample points along the direction to find the closest boundary point
        double max_search_distance = 10.0; // meters
        int max_steps = static_cast<int>(max_search_distance / map_resolution_);
        double min_distance = std::numeric_limits<double>::max();
        
        for (int step = 0; step <= max_steps; step += 2) { // Sample every 2 pixels for efficiency
            int check_x = start_x + static_cast<int>(direction.x * step);
            int check_y = start_y - static_cast<int>(direction.y * step); // Y is flipped
            
            // Check bounds
            if (check_x < 0 || check_x >= map_width_ || check_y < 0 || check_y >= map_height_) {
                break;
            }
            
            // Get distance from distance transform map
            float transform_distance = distance_map.at<float>(check_y, check_x);
            
            if (transform_distance > 0) {
                // Calculate actual distance from start point to boundary
                double step_distance = step * map_resolution_;
                double actual_distance = step_distance + transform_distance * map_resolution_;
                min_distance = std::min(min_distance, actual_distance);
            } else {
                // We hit a boundary directly
                double actual_distance = step * map_resolution_;
                min_distance = std::min(min_distance, actual_distance);
                break;
            }
        }
        
        return (min_distance == std::numeric_limits<double>::max()) ? max_search_distance : min_distance;
    }
    
    std::vector<Point2D> calculateEdgePointsWithDistances(const std::vector<Point2D>& centerline, 
                                                         const std::vector<double>& boundary_distances, 
                                                         bool is_left)
    {
        std::vector<Point2D> edge_points;
        if (centerline.size() != boundary_distances.size() || centerline.size() < 2) {
            return edge_points;
        }
        
        for (size_t i = 0; i < centerline.size(); ++i) {
            Point2D current = centerline[i];
            Point2D direction;
            
            // Calculate direction vector using neighboring points
            if (i == 0) {
                Point2D prev = centerline.back(); // Last point (closed loop)
                Point2D next = centerline[i + 1];
                Point2D dir1 = current - prev;
                Point2D dir2 = next - current;
                direction = (dir1 + dir2) * 0.5;
            } else if (i == centerline.size() - 1) {
                Point2D prev = centerline[i - 1];
                Point2D next = centerline[0]; // First point (closed loop)
                Point2D dir1 = current - prev;
                Point2D dir2 = next - current;
                direction = (dir1 + dir2) * 0.5;
            } else {
                Point2D prev = centerline[i - 1];
                Point2D next = centerline[i + 1];
                Point2D dir1 = current - prev;
                Point2D dir2 = next - current;
                direction = (dir1 + dir2) * 0.5;
            }
            
            // Normalize direction vector
            double norm = direction.norm();
            if (norm > 0) {
                direction = direction * (1.0 / norm);
            }
            
            // Calculate perpendicular vector
            Point2D perpendicular;
            if (is_left) {
                perpendicular = Point2D(-direction.y, direction.x); // Rotate 90 degrees counter-clockwise
            } else {
                perpendicular = Point2D(direction.y, -direction.x); // Rotate 90 degrees clockwise
            }
            
            // Use actual boundary distance (with small safety margin)
            double safety_margin = 0.1; // 10cm safety margin
            double edge_distance = std::max(0.1, boundary_distances[i] - safety_margin);
            
            // Calculate edge point
            Point2D edge_point = current + perpendicular * edge_distance;
            edge_points.push_back(edge_point);
        }
        
        return edge_points;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinCurvaturePlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}