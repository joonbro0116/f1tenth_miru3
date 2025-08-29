#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp/osqp.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// For GUI file dialog
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

using namespace std;

class MPC : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;

    double current_x_, current_y_, current_yaw_;
    double current_velocity_;
    bool pose_received_;

    // MPC parameters
    int prediction_horizon_;
    int n_states_;
    int n_inputs_;
    double dt_;
    double wheelbase_;
    double max_velocity_;
    double max_steering_angle_;
    double max_acceleration_;
    double min_acceleration_;
    
    // Reference path loaded from raceline optimization output
    vector<pair<double, double>> reference_path_;
    vector<double> reference_velocities_;
    string raceline_csv_path_;
    
    // OSQP workspace and data
    OSQPWorkspace* osqp_workspace_;
    OSQPData* osqp_data_;
    OSQPSettings* osqp_settings_;
    bool osqp_initialized_;
    
    // Weight matrices
    Eigen::Matrix4d Q_;  // State cost matrix
    Eigen::Matrix4d Qf_; // Terminal state cost matrix
    Eigen::Matrix2d R_;  // Input cost matrix
    
    // QP problem dimensions
    int n_vars_;         // Total optimization variables
    int n_constraints_;  // Total constraints
    
    // Vehicle state: [x, y, yaw, velocity]
    struct VehicleState {
        double x, y, yaw, velocity;
        
        Eigen::Vector4d toVector() const {
            return Eigen::Vector4d(x, y, yaw, velocity);
        }
        
        static VehicleState fromVector(const Eigen::Vector4d& vec) {
            VehicleState state;
            state.x = vec(0);
            state.y = vec(1);
            state.yaw = vec(2);
            state.velocity = vec(3);
            return state;
        }
    };

public:
    MPC() : Node("mpc_node"), pose_received_(false), osqp_initialized_(false)
    {
        // Initialize MPC parameters
        prediction_horizon_ = 10;
        n_states_ = 4;  // [x, y, yaw, velocity]
        n_inputs_ = 2;  // [steering_angle, acceleration]
        dt_ = 0.1;
        wheelbase_ = 0.33; // F1TENTH wheelbase
        max_velocity_ = 5.0;
        max_steering_angle_ = 0.34; // ~20 degrees
        max_acceleration_ = 3.0;
        min_acceleration_ = -3.0;
        
        // Calculate QP problem dimensions
        n_vars_ = prediction_horizon_ * n_inputs_;  // Only optimize control inputs
        n_constraints_ = prediction_horizon_ * n_states_ + // Dynamics constraints
                        prediction_horizon_ * 2 * n_inputs_ + // Input bound constraints  
                        n_states_; // Initial condition constraint
        
        // Initialize weight matrices
        Q_ = Eigen::Matrix4d::Identity();
        Q_(0,0) = 10.0;  // x position weight
        Q_(1,1) = 10.0;  // y position weight  
        Q_(2,2) = 1.0;   // yaw weight
        Q_(3,3) = 1.0;   // velocity weight
        
        Qf_ = Q_ * 10.0; // Terminal cost (higher weight)
        
        R_ = Eigen::Matrix2d::Identity();
        R_(0,0) = 0.1;   // steering weight
        R_(1,1) = 0.1;   // acceleration weight
        
        // Create subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pf/pose/odom", 10, std::bind(&MPC::pose_callback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MPC::odom_callback, this, std::placeholders::_1));

        // Create publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/mpc_trajectory", 10);

        // Initialize reference path (simple circular path for demonstration)
        // Select raceline CSV file using GUI dialog
        select_raceline_file();
        load_raceline_from_csv();
        
        // Initialize OSQP
        setup_osqp();

        RCLCPP_INFO(this->get_logger(), "MPC Node initialized");
    }

    void setup_osqp()
    {
        // Allocate OSQP structures
        osqp_data_ = new OSQPData;
        osqp_settings_ = new OSQPSettings;
        
        // Set default settings
        osqp_set_default_settings(osqp_settings_);
        osqp_settings_->verbose = 0;  // Suppress output
        osqp_settings_->warm_start = 1; // Enable warm start
        osqp_settings_->max_iter = 100;
        osqp_settings_->eps_abs = 1e-4;
        osqp_settings_->eps_rel = 1e-4;
        
        osqp_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "OSQP workspace initialized");
    }

    void select_raceline_file()
    {
        // Create Python script to show file dialog
        string python_script = R"(
import tkinter as tk
from tkinter import filedialog
import os
import sys

# Create root window but hide it
root = tk.Tk()
root.withdraw()

# Open file dialog
file_path = filedialog.askopenfilename(
    title="Select Raceline CSV File",
    initialdir=os.path.expanduser("~"),
    filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
)

if file_path:
    print(file_path)
else:
    # Use default path if no file selected
    print("")

root.destroy()
)";
        
        // Write Python script to temporary file
        string temp_script = "/tmp/raceline_selector.py";
        ofstream script_file(temp_script);
        script_file << python_script;
        script_file.close();
        
        // Execute Python script and capture output
        string command = "python3 " + temp_script;
        FILE* pipe = popen(command.c_str(), "r");
        if (pipe) {
            char buffer[512];
            if (fgets(buffer, sizeof(buffer), pipe)) {
                raceline_csv_path_ = string(buffer);
                // Remove newline character
                if (!raceline_csv_path_.empty() && raceline_csv_path_.back() == '\n') {
                    raceline_csv_path_.pop_back();
                }
            }
            pclose(pipe);
        }
        
        // Clean up temporary file
        remove(temp_script.c_str());
        
        // If no file selected, use default fallback
        if (raceline_csv_path_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No raceline file selected. Using default circular path.");
            generate_default_reference_path();
        } else {
            RCLCPP_INFO(this->get_logger(), "Selected raceline file: %s", raceline_csv_path_.c_str());
        }
    }
    
    void load_raceline_from_csv()
    {
        if (raceline_csv_path_.empty()) {
            return;
        }
        
        reference_path_.clear();
        reference_velocities_.clear();
        
        ifstream file(raceline_csv_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open raceline CSV file: %s", raceline_csv_path_.c_str());
            generate_default_reference_path();
            return;
        }
        
        string line;
        int line_count = 0;
        while (getline(file, line)) {
            line_count++;
            
            // Skip empty lines
            if (line.empty()) continue;
            
            // Parse CSV line (x,y,velocity format)
            stringstream ss(line);
            string x_str, y_str, v_str;
            
            if (getline(ss, x_str, ',') && getline(ss, y_str, ',') && getline(ss, v_str, ',')) {
                try {
                    double x = stod(x_str);
                    double y = stod(y_str);
                    double v = stod(v_str);
                    
                    reference_path_.push_back({x, y});
                    reference_velocities_.push_back(v);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse line %d: %s", line_count, line.c_str());
                }
            }
        }
        file.close();
        
        if (reference_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid waypoints loaded from CSV file");
            generate_default_reference_path();
        } else {
            RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from raceline CSV", reference_path_.size());
        }
    }
    
    void generate_default_reference_path()
    {
        reference_path_.clear();
        reference_velocities_.clear();
        
        double radius = 3.0;
        int num_points = 100;
        double default_velocity = 2.0; // m/s
        
        for (int i = 0; i < num_points; i++) {
            double angle = 2.0 * M_PI * i / num_points;
            double x = radius * cos(angle);
            double y = radius * sin(angle);
            reference_path_.push_back({x, y});
            reference_velocities_.push_back(default_velocity);
        }
    }
    
    // Get linearized dynamics matrices around operating point
    pair<Eigen::Matrix4d, Eigen::Matrix<double,4,2>> get_linearized_dynamics(
        const VehicleState& state, const Eigen::Vector2d& u_op)
    {
        double yaw = state.yaw;
        double v = state.velocity;
        double steering = u_op(0);
        
        // Discrete-time A matrix: x[k+1] = A*x[k] + B*u[k] + c
        Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
        A(0,2) = -dt_ * v * sin(yaw);     // dx/dyaw
        A(0,3) = dt_ * cos(yaw);          // dx/dv
        A(1,2) = dt_ * v * cos(yaw);      // dy/dyaw  
        A(1,3) = dt_ * sin(yaw);          // dy/dv
        A(2,3) = dt_ * tan(steering) / wheelbase_; // dyaw/dv
        
        // Discrete-time B matrix
        Eigen::Matrix<double,4,2> B;
        B.setZero();
        B(2,0) = dt_ * v / (wheelbase_ * cos(steering) * cos(steering)); // dyaw/dsteering
        B(3,1) = dt_;  // dv/daccel
        
        return {A, B};
    }
    
    // Build QP matrices for OSQP
    struct QPMatrices {
        Eigen::SparseMatrix<double> P;  // Hessian matrix
        Eigen::VectorXd q;              // Linear term
        Eigen::SparseMatrix<double> A;  // Constraint matrix
        Eigen::VectorXd l;              // Lower bounds
        Eigen::VectorXd u;              // Upper bounds
    };
    
    QPMatrices build_qp_matrices(const VehicleState& current_state, 
                                 const vector<VehicleState>& reference_trajectory)
    {
        QPMatrices qp;
        
        // Initialize matrices
        qp.P.resize(n_vars_, n_vars_);
        qp.q.resize(n_vars_);
        qp.A.resize(n_constraints_, n_vars_);
        qp.l.resize(n_constraints_);
        qp.u.resize(n_constraints_);
        
        // Build cost matrices
        vector<Eigen::Triplet<double>> P_triplets;
        qp.q.setZero();
        
        // For simplicity, we'll use input regularization only
        // Full implementation would include state tracking costs
        for (int k = 0; k < prediction_horizon_; k++) {
            int u_start = k * n_inputs_;
            
            // Input cost: u^T * R * u
            for (int i = 0; i < n_inputs_; i++) {
                P_triplets.push_back(Eigen::Triplet<double>(
                    u_start + i, u_start + i, R_(i,i)));
            }
        }
        qp.P.setFromTriplets(P_triplets.begin(), P_triplets.end());
        
        // Build constraint matrices
        vector<Eigen::Triplet<double>> A_triplets;
        int constraint_idx = 0;
        
        // 1. Dynamics constraints: x[k+1] = A*x[k] + B*u[k]
        // We'll linearize around current operating point
        VehicleState state = current_state;
        Eigen::Vector2d u_op(0.0, 0.0); // Operating point for linearization
        
        for (int k = 0; k < prediction_horizon_; k++) {
            auto [A_d, B_d] = get_linearized_dynamics(state, u_op);
            
            // Predict next state for next linearization point
            if (k < reference_trajectory.size()) {
                state = reference_trajectory[k];
            }
            
            // For each state dimension
            for (int i = 0; i < n_states_; i++) {
                int row = constraint_idx + i;
                int u_col = k * n_inputs_;
                
                // Add B matrix entries: B*u terms
                for (int j = 0; j < n_inputs_; j++) {
                    if (abs(B_d(i,j)) > 1e-8) {
                        A_triplets.push_back(Eigen::Triplet<double>(
                            row, u_col + j, B_d(i,j)));
                    }
                }
                
                // Set constraint bounds based on reference trajectory
                if (k < reference_trajectory.size()) {
                    // x[k+1] = A*x[k] + B*u[k]
                    // Rearrange: B*u[k] = x[k+1] - A*x[k]
                    Eigen::Vector4d x_next = reference_trajectory[k].toVector();
                    Eigen::Vector4d x_curr = (k == 0) ? current_state.toVector() : 
                                            reference_trajectory[k-1].toVector();
                    Eigen::Vector4d rhs = x_next - A_d * x_curr;
                    
                    qp.l(row) = rhs(i);
                    qp.u(row) = rhs(i);
                } else {
                    qp.l(row) = 0.0;
                    qp.u(row) = 0.0;
                }
            }
            constraint_idx += n_states_;
        }
        
        // 2. Input bound constraints
        for (int k = 0; k < prediction_horizon_; k++) {
            int u_start = k * n_inputs_;
            
            // Steering angle bounds
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_idx, u_start, 1.0)); // steering
            qp.l(constraint_idx) = -max_steering_angle_;
            qp.u(constraint_idx) = max_steering_angle_;
            constraint_idx++;
            
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_idx, u_start, -1.0)); // -steering  
            qp.l(constraint_idx) = -max_steering_angle_;
            qp.u(constraint_idx) = max_steering_angle_;
            constraint_idx++;
            
            // Acceleration bounds
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_idx, u_start + 1, 1.0)); // acceleration
            qp.l(constraint_idx) = min_acceleration_;
            qp.u(constraint_idx) = max_acceleration_;
            constraint_idx++;
            
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_idx, u_start + 1, -1.0)); // -acceleration
            qp.l(constraint_idx) = min_acceleration_;
            qp.u(constraint_idx) = max_acceleration_;
            constraint_idx++;
        }
        
        qp.A.setFromTriplets(A_triplets.begin(), A_triplets.end());
        
        return qp;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr &pose_msg)
    {
        current_x_ = pose_msg->pose.position.x;
        current_y_ = pose_msg->pose.position.y;
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            pose_msg->pose.orientation.x,
            pose_msg->pose.orientation.y,
            pose_msg->pose.orientation.z,
            pose_msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        
        pose_received_ = true;
        
        // Run MPC optimization
        if (pose_received_) {
            run_mpc();
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr &odom_msg)
    {
        current_velocity_ = sqrt(pow(odom_msg->twist.twist.linear.x, 2) + 
                                pow(odom_msg->twist.twist.linear.y, 2));
    }

    pair<int, double> find_closest_waypoint()
    {
        double min_distance = std::numeric_limits<double>::max();
        int closest_idx = 0;
        
        for (int i = 0; i < reference_path_.size(); i++) {
            double dx = current_x_ - reference_path_[i].first;
            double dy = current_y_ - reference_path_[i].second;
            double distance = sqrt(dx*dx + dy*dy);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }
        
        return {closest_idx, min_distance};
    }

    void run_mpc()
    {
        if (!osqp_initialized_) return;
        
        auto [closest_idx, distance] = find_closest_waypoint();
        
        // Get reference trajectory
        vector<VehicleState> reference_trajectory;
        for (int i = 0; i < prediction_horizon_; i++) {
            int idx = (closest_idx + i + 1) % reference_path_.size();
            VehicleState ref_state;
            ref_state.x = reference_path_[idx].first;
            ref_state.y = reference_path_[idx].second;
            ref_state.yaw = 0.0; // Could compute from path
            ref_state.velocity = 2.0; // Target velocity
            reference_trajectory.push_back(ref_state);
        }
        
        // Solve MPC with OSQP
        auto [steering, acceleration] = solve_mpc_osqp(reference_trajectory);
        
        // Publish control command
        publish_control_command(steering, max(0.0, current_velocity_ + acceleration * dt_));
        
        // Visualize predicted trajectory
        visualize_trajectory(steering, acceleration);
    }
    
    pair<double, double> solve_mpc_osqp(const vector<VehicleState>& reference_trajectory)
    {
        VehicleState current_state = {current_x_, current_y_, current_yaw_, current_velocity_};
        
        // Build QP matrices
        auto qp = build_qp_matrices(current_state, reference_trajectory);
        
        // Convert Eigen sparse matrices to OSQP format
        csc* P_csc = eigen_to_csc(qp.P);
        csc* A_csc = eigen_to_csc(qp.A);
        
        // Setup OSQP data
        osqp_data_->n = n_vars_;
        osqp_data_->m = n_constraints_;
        osqp_data_->P = P_csc;
        osqp_data_->A = A_csc;
        osqp_data_->q = qp.q.data();
        osqp_data_->l = qp.l.data();
        osqp_data_->u = qp.u.data();
        
        // Setup workspace if not already done
        static bool workspace_setup = false;
        if (!workspace_setup) {
            osqp_setup(&osqp_workspace_, osqp_data_, osqp_settings_);
            workspace_setup = true;
        } else {
            // Update matrices for warm start
            osqp_update_P(osqp_workspace_, P_csc->x, OSQP_NULL, P_csc->nzmax);
            osqp_update_A(osqp_workspace_, A_csc->x, OSQP_NULL, A_csc->nzmax);
            osqp_update_lin_cost(osqp_workspace_, osqp_data_->q);
            osqp_update_bounds(osqp_workspace_, osqp_data_->l, osqp_data_->u);
        }
        
        // Solve QP problem
        osqp_solve(osqp_workspace_);
        
        // Extract solution
        double steering = 0.0;
        double acceleration = 0.0;
        
        if (osqp_workspace_->info->status_val == OSQP_SOLVED || 
            osqp_workspace_->info->status_val == OSQP_SOLVED_INACCURATE) {
            
            // Get first control input (receding horizon)
            steering = osqp_workspace_->solution->x[0];        // First steering
            acceleration = osqp_workspace_->solution->x[1];    // First acceleration
            
            RCLCPP_INFO(this->get_logger(), "OSQP solved: steering=%.3f, accel=%.3f", 
                        steering, acceleration);
        } else {
            RCLCPP_WARN(this->get_logger(), "OSQP failed to solve, status: %d", 
                        osqp_workspace_->info->status_val);
            // Use safe default values
            steering = 0.0;
            acceleration = 0.0;
        }
        
        // Clean up temporary matrices
        csc_spfree(P_csc);
        csc_spfree(A_csc);
        
        return {steering, acceleration};
    }
    
    // Helper function to convert Eigen sparse matrix to OSQP csc format
    csc* eigen_to_csc(const Eigen::SparseMatrix<double>& eigen_mat) {
        csc* csc_mat = csc_matrix(eigen_mat.rows(), eigen_mat.cols(), 
                                  eigen_mat.nonZeros(), 
                                  const_cast<double*>(eigen_mat.valuePtr()),
                                  const_cast<c_int*>(eigen_mat.innerIndexPtr()),
                                  const_cast<c_int*>(eigen_mat.outerIndexPtr()));
        return csc_mat;
    }

    void publish_control_command(double steering_angle, double velocity)
    {
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";
        
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = velocity;
        
        drive_pub_->publish(drive_msg);
        
        RCLCPP_INFO(this->get_logger(), "Published: steering=%.3f, speed=%.3f", 
                    steering_angle, velocity);
    }

    void visualize_trajectory(double steering_angle, double acceleration)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.scale.x = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        // Predict and add trajectory points
        VehicleState current_state = {current_x_, current_y_, current_yaw_, current_velocity_};
        
        for (int i = 0; i < prediction_horizon_; i++) {
            double velocity = max(0.0, current_state.velocity + acceleration * dt_);
            
            // Predict next state using bicycle model
            current_state.x += velocity * cos(current_state.yaw) * dt_;
            current_state.y += velocity * sin(current_state.yaw) * dt_;
            current_state.yaw += velocity * tan(steering_angle) / wheelbase_ * dt_;
            current_state.velocity = velocity;
            
            geometry_msgs::msg::Point point;
            point.x = current_state.x;
            point.y = current_state.y;
            point.z = 0.0;
            marker.points.push_back(point);
        }
        
        viz_pub_->publish(marker);
    }

    ~MPC() {
        if (osqp_workspace_) {
            osqp_cleanup(osqp_workspace_);
        }
        if (osqp_data_) {
            delete osqp_data_;
        }
        if (osqp_settings_) {
            delete osqp_settings_;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPC>());
    rclcpp::shutdown();
    return 0;
}