"""
FIXED VERSION of F1TENTH trajectory optimization
Main fixes:
1. Proper normal vector calculation
2. Stable optimization function
3. Clean visualization
4. Robust curvature calculation
"""
import numpy as np
import time
import json
import os
import sys
import matplotlib.pyplot as plt
import configparser
from datetime import datetime
import argparse
import shutil
from pathlib import Path
from tkinter import Tk
from tkinter.filedialog import askopenfilename, asksaveasfilename

# Try to import scipy for optimization and splines
try:
    from scipy.optimize import minimize
    from scipy.interpolate import splprep, splev, interp1d
    SCIPY_AVAILABLE = True
    SPLINE_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    SPLINE_AVAILABLE = False
    print("WARNING: scipy not available, using simple optimization")

def select_centerline_file(default_path=None):
    """팝업으로 centerline with boundary CSV 파일을 선택합니다."""
    if default_path is None:
        root = Tk()
        root.withdraw()  # Hide the main window
        file_path = askopenfilename(
            title='Centerline with Boundary CSV 파일 선택',
            filetypes=[('CSV Files', '*.csv'), ('All Files', '*.*')],
            initialdir='/home/f1/f1tenth_ws/path_generate/centerline_with_boundary'
        )
        if not file_path:
            sys.exit("Centerline 파일이 선택되지 않았습니다.")
        return file_path
    return default_path

def select_output_file(default_path=None):
    """팝업으로 출력 raceline CSV 파일 경로를 선택합니다."""
    if default_path is None:
        root = Tk()
        root.withdraw()  # Hide the main window
        file_path = asksaveasfilename(
            title='Raceline CSV 파일 저장 위치 선택',
            filetypes=[('CSV Files', '*.csv'), ('All Files', '*.*')],
            initialdir='/home/f1/f1tenth_ws/path_generate/raceline',
            defaultextension='.csv'
        )
        if not file_path:
            sys.exit("출력 파일 경로가 선택되지 않았습니다.")
        return file_path
    return default_path

# Create the parser
parser = argparse.ArgumentParser(description='Generate optimal trajectory for F1TENTH racing.')
parser.add_argument('--centerline_path', type=str, help='Path to the centerline CSV file with boundaries')
parser.add_argument('--output_path', type=str, help='Path to save the output raceline')
parser.add_argument('--show_plot', action='store_true', help='시각화 창 표시')
parser.add_argument('--use_popup', action='store_true', help='팝업 다이얼로그로 파일 선택')

args = parser.parse_args()
SCRIPT_DIR = Path(__file__).parent.parent

# Vehicle parameters
veh_params = {
    "length": 0.58,      
    "width": 0.31,       
    "wheelbase": 0.33,   
    "mass": 3.47,        
    "v_max": 6.0,        
    "curvlim": 0.2,      
    "dragcoeff": 0.1     
}

optim_opts = {
    "width_opt": veh_params["width"] + 0.1,
    "safety_margin": 0.3,  # Reduced safety margin
    "max_iterations": 100
}

def load_centerline_with_boundaries(file_path):
    """Load centerline with boundary data from CSV file"""
    try:
        data = np.loadtxt(file_path, delimiter=',', skiprows=1)
        
        if data.shape[1] == 4:
            # Format: center_x, center_y, left_distance, right_distance
            center_x = data[:, 0]
            center_y = data[:, 1] 
            left_dist = data[:, 2]
            right_dist = data[:, 3]
            
            # Create reftrack format: [x, y, w_tr_right, w_tr_left]
            reftrack = np.column_stack((center_x, center_y, right_dist, left_dist))
            
        elif data.shape[1] >= 2:
            # Simple centerline format: x, y
            center_x = data[:, 0]
            center_y = data[:, 1]
            
            # Assume fixed track width
            default_width = 1.5  # meters
            reftrack = np.column_stack((center_x, center_y, 
                                      np.full_like(center_x, default_width),
                                      np.full_like(center_x, default_width)))
        else:
            raise ValueError(f"Invalid CSV format. Expected at least 2 columns, got {data.shape[1]}")
            
        print(f"Loaded centerline: {len(reftrack)} points")
        print(f"Track width range: {np.min(reftrack[:, 2:]):.2f}m - {np.max(reftrack[:, 2:]):.2f}m")
        
        return reftrack
        
    except Exception as e:
        print(f"Error loading centerline: {e}")
        return None

def calculate_track_normals_fixed(reftrack):
    """FIXED: Calculate normalized normal vectors for each point on the track"""
    n_points = len(reftrack)
    normals = np.zeros((n_points, 2))
    
    # Calculate tangent vectors first
    tangents = np.zeros((n_points, 2))
    
    for i in range(n_points):
        if i == 0:
            # Forward difference for first point
            tangents[i] = reftrack[1, :2] - reftrack[0, :2]
        elif i == n_points - 1:
            # Backward difference for last point  
            tangents[i] = reftrack[-1, :2] - reftrack[-2, :2]
        else:
            # Central difference for interior points
            tangents[i] = reftrack[i+1, :2] - reftrack[i-1, :2]
    
    # Normalize tangent vectors and compute normals
    for i in range(n_points):
        # Normalize tangent
        norm = np.linalg.norm(tangents[i])
        if norm > 1e-10:
            tangents[i] /= norm
            
            # Normal vector (rotate tangent 90 degrees CCW)
            # Left is positive direction
            normals[i, 0] = -tangents[i, 1] 
            normals[i, 1] = tangents[i, 0]
        else:
            # Handle degenerate case
            if i > 0:
                normals[i] = normals[i-1]
            else:
                normals[i] = np.array([0, 1])
                
    return normals

def calculate_curvature_smooth(path):
    """SMOOTH curvature calculation for racing lines"""
    n_points = len(path)
    curvature = np.zeros(n_points)
    
    # Use larger window for smoother calculation
    window = min(7, n_points // 10)  # Adaptive window size
    if window < 3:
        window = 3
    
    for i in range(window, n_points - window):
        # Use points further apart for smoother curvature
        p1 = path[i - window]
        p2 = path[i]
        p3 = path[i + window]
        
        # Calculate curvature using three-point method
        v1 = p2 - p1
        v2 = p3 - p2
        
        # Cross and dot products
        cross = np.cross(v1, v2)
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        
        if v1_norm > 1e-8 and v2_norm > 1e-8:
            # Simplified curvature calculation
            sin_theta = cross / (v1_norm * v2_norm)
            avg_segment = (v1_norm + v2_norm) / 2
            
            if avg_segment > 1e-8:
                curvature[i] = sin_theta / avg_segment
    
    # Handle boundaries with extrapolation
    for i in range(window):
        curvature[i] = curvature[window]
    for i in range(n_points - window, n_points):
        curvature[i] = curvature[n_points - window - 1]
    
    # HEAVY smoothing for racing line
    for _ in range(8):  # More smoothing passes
        curvature_smooth = np.copy(curvature)
        for i in range(3, n_points - 3):
            # Wider smoothing kernel
            curvature_smooth[i] = (
                0.4 * curvature[i] + 
                0.15 * (curvature[i-1] + curvature[i+1]) +
                0.1 * (curvature[i-2] + curvature[i+2]) +
                0.05 * (curvature[i-3] + curvature[i+3])
            )
        curvature = curvature_smooth
    
    return curvature

def racing_line_optimization(reftrack, safety_margin=0.2):
    """AGGRESSIVE minimum curvature optimization"""
    print("Running AGGRESSIVE minimum curvature optimization...")
    
    n_points = len(reftrack)
    normals = calculate_track_normals_fixed(reftrack)
    
    # For very large tracks, still use geometric but more aggressive
    if n_points > 300:
        print(f"Large track ({n_points} points), using aggressive geometric optimization")
        return aggressive_geometric_racing_line(reftrack, normals, safety_margin)
    
    if not SCIPY_AVAILABLE:
        print("Scipy not available, using aggressive geometric optimization")
        return aggressive_geometric_racing_line(reftrack, normals, safety_margin)
    
    # Define bounds for alpha (lateral offset) - MORE AGGRESSIVE
    bounds = []
    for i in range(n_points):
        w_left = reftrack[i, 3] - safety_margin   # Smaller safety margin
        w_right = reftrack[i, 2] - safety_margin
        bounds.append((-w_right, w_left))
    
    def pure_minimum_curvature_objective(alpha):
        """PURE minimum curvature objective - no compromise"""
        # Create raceline
        raceline = np.zeros((n_points, 2))
        for i in range(n_points):
            raceline[i] = reftrack[i, :2] + alpha[i] * normals[i]
        
        # Calculate curvature
        curvature = calculate_curvature_smooth(raceline)
        
        # PURE minimum curvature: minimize sum of squared curvatures
        curvature_cost = np.sum(curvature**2)
        
        # Minimal smoothness penalty (much smaller)
        smoothness_penalty = 0.01 * np.sum(np.diff(alpha)**2)  # Reduced from 0.1
        
        return curvature_cost + smoothness_penalty
    
    # More aggressive initial guesses
    best_result = None
    best_objective = float('inf')
    
    for attempt in range(3):
        if attempt == 0:
            # Start with aggressive geometric solution
            alpha_attempt = aggressive_geometric_racing_line(reftrack, normals, safety_margin)
        elif attempt == 1:
            # Start with wider random perturbation
            alpha_attempt = np.random.normal(0, 0.3, n_points)  # Increased from 0.1
            for i in range(n_points):
                w_left = reftrack[i, 3] - safety_margin
                w_right = reftrack[i, 2] - safety_margin
                alpha_attempt[i] = np.clip(alpha_attempt[i], -w_right, w_left)
        else:
            # Start from centerline
            alpha_attempt = np.zeros(n_points)
        
        try:
            result = minimize(
                pure_minimum_curvature_objective,
                alpha_attempt,
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': 100, 'ftol': 1e-12}  # More precise convergence
            )
            
            if result.fun < best_objective:
                best_result = result
                best_objective = result.fun
                
        except Exception as e:
            print(f"Optimization attempt {attempt+1} failed: {e}")
            continue
    
    if best_result is not None:
        print(f"Pure minimum curvature converged with objective: {best_result.fun:.6f}")
        return best_result.x
    else:
        print("All optimization attempts failed, using aggressive geometric method")
        return aggressive_geometric_racing_line(reftrack, normals, safety_margin)

def smooth_raceline_bspline(raceline, smoothing_factor=0.1, num_points=None):
    """Ultra-smooth B-spline smoothing for raceline"""
    if not SPLINE_AVAILABLE:
        print("B-spline not available, using aggressive smoothing")
        return smooth_raceline_aggressive(raceline, iterations=15)
    
    n_points = len(raceline)
    if num_points is None:
        num_points = n_points
    
    # Check if closed loop
    is_closed_loop = np.linalg.norm(raceline[0] - raceline[-1]) < 1.0
    
    try:
        if is_closed_loop:
            # For closed loops, add wraparound points
            extended_raceline = np.vstack([raceline[-10:], raceline, raceline[:10]])
            x, y = extended_raceline[:, 0], extended_raceline[:, 1]
            
            # Parameterize by cumulative distance
            distances = np.zeros(len(x))
            for i in range(1, len(x)):
                distances[i] = distances[i-1] + np.linalg.norm([x[i] - x[i-1], y[i] - y[i-1]])
            
            # Fit B-spline
            tck, u = splprep([x, y], u=distances, s=smoothing_factor * n_points, per=1, k=3)
            
            # Evaluate spline at original parameter values
            u_original = distances[10:-10]  # Remove extended parts
            x_smooth, y_smooth = splev(u_original, tck)
            
            smoothed = np.column_stack([x_smooth, y_smooth])
            
        else:
            # For open tracks
            x, y = raceline[:, 0], raceline[:, 1]
            
            # Parameterize by cumulative distance
            distances = np.zeros(len(x))
            for i in range(1, len(x)):
                distances[i] = distances[i-1] + np.linalg.norm([x[i] - x[i-1], y[i] - y[i-1]])
            
            # Fit B-spline
            tck, u = splprep([x, y], u=distances, s=smoothing_factor * n_points, k=3)
            
            # Evaluate at uniform intervals
            u_new = np.linspace(0, distances[-1], num_points)
            x_smooth, y_smooth = splev(u_new, tck)
            
            smoothed = np.column_stack([x_smooth, y_smooth])
        
        print(f"B-spline smoothing applied with factor {smoothing_factor}")
        return smoothed
        
    except Exception as e:
        print(f"B-spline smoothing failed: {e}, using fallback")
        return smooth_raceline_aggressive(raceline, iterations=15)

def smooth_raceline_aggressive(raceline, iterations=10):
    """SAFE aggressive smoothing for CLOSED LOOP raceline"""
    smoothed = np.copy(raceline)
    n_points = len(raceline)
    
    # Check if this is a closed loop (start and end points are close)
    is_closed_loop = np.linalg.norm(raceline[0] - raceline[-1]) < 1.0  # Within 1 meter
    
    def get_safe_circular_index(i, n):
        """Get circular index for closed loop with bounds checking"""
        if n <= 0:
            return 0
        return i % n
    
    # SAFE smoothing with bounds checking
    for iteration in range(iterations):
        prev_smoothed = np.copy(smoothed)
        new_smoothed = np.copy(smoothed)  # Create separate array to avoid in-place modification issues
        
        for i in range(n_points):
            try:
                if is_closed_loop:
                    # Use circular indexing for closed loop with safety checks
                    i_m2 = get_safe_circular_index(i - 2, n_points)
                    i_m1 = get_safe_circular_index(i - 1, n_points)
                    i_p1 = get_safe_circular_index(i + 1, n_points)
                    i_p2 = get_safe_circular_index(i + 2, n_points)
                    
                    # Verify indices are valid
                    indices = [i_m2, i_m1, i, i_p1, i_p2]
                    if all(0 <= idx < n_points for idx in indices):
                        # 5-point smoothing kernel
                        new_smoothed[i] = (
                            0.3 * prev_smoothed[i] +
                            0.25 * (prev_smoothed[i_m1] + prev_smoothed[i_p1]) +
                            0.1 * (prev_smoothed[i_m2] + prev_smoothed[i_p2])
                        )
                    else:
                        # Fallback: keep original point
                        new_smoothed[i] = prev_smoothed[i]
                else:
                    # Safe boundary handling for open tracks
                    if 2 <= i <= n_points - 3:
                        new_smoothed[i] = (
                            0.3 * prev_smoothed[i] +
                            0.25 * (prev_smoothed[i-1] + prev_smoothed[i+1]) +
                            0.1 * (prev_smoothed[i-2] + prev_smoothed[i+2])
                        )
            except (IndexError, ValueError) as e:
                # If any error occurs, keep the original point
                new_smoothed[i] = prev_smoothed[i]
                
        smoothed = new_smoothed
    
    # Minimal additional smoothing to avoid over-processing
    for _ in range(2):  # Reduced from 3 to 2
        prev_smoothed = np.copy(smoothed)
        new_smoothed = np.copy(smoothed)
        
        for i in range(n_points):
            try:
                if is_closed_loop:
                    # Smaller kernel for final smoothing
                    i_m1 = get_safe_circular_index(i - 1, n_points)
                    i_p1 = get_safe_circular_index(i + 1, n_points)
                    
                    if 0 <= i_m1 < n_points and 0 <= i_p1 < n_points:
                        new_smoothed[i] = (
                            0.5 * prev_smoothed[i] +
                            0.25 * (prev_smoothed[i_m1] + prev_smoothed[i_p1])
                        )
                else:
                    if 1 <= i <= n_points - 2:
                        new_smoothed[i] = (
                            0.5 * prev_smoothed[i] +
                            0.25 * (prev_smoothed[i-1] + prev_smoothed[i+1])
                        )
            except (IndexError, ValueError):
                new_smoothed[i] = prev_smoothed[i]
                
        smoothed = new_smoothed
    
    # For closed loops, ensure perfect continuity at start/end
    if is_closed_loop:
        # Make the last point exactly equal to the first point
        smoothed[-1] = smoothed[0]
    
    return smoothed
def aggressive_geometric_racing_line(reftrack, normals, safety_margin):
    """조금 완만하게 조정된 geometric racing line"""
    n_points = len(reftrack)
    alpha_opt = np.zeros(n_points)
    
    is_closed_loop = np.linalg.norm(reftrack[0, :2] - reftrack[-1, :2]) < 1.0
    centerline = reftrack[:, :2]
    
    # 스무딩 강화
    centerline_smooth = smooth_raceline_bspline(centerline, smoothing_factor=0.3)
    track_curvature = calculate_curvature_smooth(centerline_smooth)
    
    for i in range(n_points):
        w_left = reftrack[i, 3] - safety_margin
        w_right = reftrack[i, 2] - safety_margin
        max_offset = min(w_left, w_right) * 0.6  # 공격성 감소 (80% -> 60%)
        
        if track_curvature[i] > 0.002:  # Left turn
            curvature_factor = np.tanh(track_curvature[i] * 3.0)  # 공격성 감소
            alpha_opt[i] = max_offset * curvature_factor
        elif track_curvature[i] < -0.002:  # Right turn
            curvature_factor = np.tanh(abs(track_curvature[i]) * 3.0)
            alpha_opt[i] = -max_offset * curvature_factor
        else:
            alpha_opt[i] = 0
    
    # 스무딩 패스 강화
    for _ in range(12):  # 이전 8 -> 12로 증가
        alpha_smooth = np.copy(alpha_opt)
        for i in range(n_points):
            if is_closed_loop:
                i_m2 = (i - 2) % n_points
                i_m1 = (i - 1) % n_points
                i_p1 = (i + 1) % n_points
                i_p2 = (i + 2) % n_points
                
                alpha_smooth[i] = (
                    0.45 * alpha_opt[i] + 
                    0.2 * (alpha_opt[i_m1] + alpha_opt[i_p1]) + 
                    0.05 * (alpha_opt[i_m2] + alpha_opt[i_p2])
                )
            else:
                if 2 <= i <= n_points - 3:
                    alpha_smooth[i] = (
                        0.45 * alpha_opt[i] + 
                        0.2 * (alpha_opt[i-1] + alpha_opt[i+1]) +
                        0.05 * (alpha_opt[i-2] + alpha_opt[i+2])
                    )
        alpha_opt = alpha_smooth
        if is_closed_loop:
            alpha_opt[-1] = alpha_opt[0]
    
    return alpha_opt

def create_raceline(reftrack, alpha_opt):
    """Create AGGRESSIVE raceline with pure minimum curvature focus"""
    normals = calculate_track_normals_fixed(reftrack)
    n_points = len(reftrack)
    raceline = np.zeros((n_points, 2))
    
    # Create initial raceline with AGGRESSIVE bounds
    for i in range(n_points):
        # More aggressive bounds - smaller safety margin
        w_left = reftrack[i, 3] - 0.05   # Reduced from 0.1
        w_right = reftrack[i, 2] - 0.05
        
        # Allow larger alpha values (less clamping)
        alpha_clamped = np.clip(alpha_opt[i], -w_right, w_left)
        
        # Create raceline point
        raceline[i] = reftrack[i, :2] + alpha_clamped * normals[i]
        
        # Less restrictive safety check
        distance_from_center = np.linalg.norm(raceline[i] - reftrack[i, :2])
        max_reasonable_distance = max(w_left, w_right) * 1.2  # Reduced multiplier
        
        if distance_from_center > max_reasonable_distance:
            raceline[i] = reftrack[i, :2]
    
    # Apply B-SPLINE smoothing for ultra-smooth lines
    raceline_smooth = smooth_raceline_bspline(raceline, smoothing_factor=0.2)
    
    # Final bounds check with aggressive margins
    for i in range(n_points):
        offset_vector = raceline_smooth[i] - reftrack[i, :2]
        lateral_offset = np.dot(offset_vector, normals[i])
        
        # More aggressive final bounds
        w_left = reftrack[i, 3] - 0.08   # Increased from 0.15
        w_right = reftrack[i, 2] - 0.08
        
        # Clamp but allow more aggressive positioning
        if lateral_offset > w_left:
            lateral_offset = w_left
        elif lateral_offset < -w_right:
            lateral_offset = -w_right
            
        raceline_smooth[i] = reftrack[i, :2] + lateral_offset * normals[i]
    
    return raceline_smooth

def calculate_velocity_profile(raceline, kappa, v_max=6.0):
    """Calculate physics-based velocity profile"""
    # Tire-road friction coefficient
    mu = 0.8
    g = 9.81
    ay_max = mu * g  # Maximum lateral acceleration
    ax_max = 4.0     # Maximum longitudinal acceleration
    
    n_points = len(raceline)
    v_profile = np.ones(n_points) * v_max
    
    # Calculate segment distances
    distances = np.zeros(n_points)
    for i in range(n_points):
        next_i = (i + 1) % n_points
        distances[i] = np.linalg.norm(raceline[next_i] - raceline[i])
    
    # Set cornering speeds based on curvature
    for i in range(n_points):
        if abs(kappa[i]) > 1e-6:
            v_curv = np.sqrt(ay_max / abs(kappa[i]))
            v_profile[i] = min(v_max, v_curv)
    
    # Forward pass (acceleration limited)
    for i in range(1, n_points):
        prev_i = i - 1
        if distances[prev_i] > 1e-6:
            v_accel = np.sqrt(v_profile[prev_i]**2 + 2 * ax_max * distances[prev_i])
            v_profile[i] = min(v_profile[i], v_accel)
    
    # Backward pass (deceleration limited)
    for i in range(n_points - 2, -1, -1):
        next_i = i + 1
        if distances[i] > 1e-6:
            v_decel = np.sqrt(v_profile[next_i]**2 + 2 * ax_max * distances[i])
            v_profile[i] = min(v_profile[i], v_decel)
    
    # Smooth velocity profile
    for _ in range(3):
        v_smooth = np.copy(v_profile)
        for i in range(1, n_points - 1):
            v_smooth[i] = 0.7 * v_profile[i] + 0.15 * (v_profile[i-1] + v_profile[i+1])
        v_profile = v_smooth
        
    return v_profile

def export_raceline_csv(raceline, velocity_profile, output_path, format_type='simple'):
    """Export raceline to CSV file"""
    
    if format_type == 'simple':
        header = "x,y"
        data = raceline
    elif format_type == 'with_velocity':
        header = "x,y,velocity"
        data = np.column_stack((raceline, velocity_profile))
    else:
        # Full trajectory format
        yaw = np.zeros(len(raceline))
        for i in range(len(raceline) - 1):
            dx = raceline[i+1, 0] - raceline[i, 0]
            dy = raceline[i+1, 1] - raceline[i, 1]
            yaw[i] = np.arctan2(dy, dx)
        yaw[-1] = yaw[-2]
        
        kappa = calculate_curvature_smooth(raceline)
        
        header = "x,y,yaw,velocity,curvature"
        data = np.column_stack((raceline, yaw, velocity_profile, kappa))
    
    np.savetxt(output_path, data, delimiter=',', header=header, comments='')
    print(f"Raceline exported to: {output_path}")

def plot_results_clean(reftrack, raceline, velocity_profile=None, save_path=None, show_plot=False):
    """CLEAN visualization without the red mess"""
    plt.ioff()
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    ax1, ax2, ax3, ax4 = axes.flatten()
    
    # Plot 1: Track layout - CLEAN VERSION
    # Plot centerline
    ax1.plot(reftrack[:, 0], reftrack[:, 1], 'b-', linewidth=2, label='Centerline', alpha=0.8)
    
    # Plot raceline - SINGLE CLEAN LINE
    ax1.plot(raceline[:, 0], raceline[:, 1], 'r-', linewidth=3, label='Optimized Raceline', alpha=0.9)
    
    # Plot track boundaries
    normals = calculate_track_normals_fixed(reftrack)
    left_boundary = reftrack[:, :2] + reftrack[:, 3:4] * normals
    right_boundary = reftrack[:, :2] - reftrack[:, 2:3] * normals
    
    ax1.plot(left_boundary[:, 0], left_boundary[:, 1], 'k--', alpha=0.5, linewidth=1, label='Track Boundaries')
    ax1.plot(right_boundary[:, 0], right_boundary[:, 1], 'k--', alpha=0.5, linewidth=1)
    
    # Start/finish marker
    ax1.plot(reftrack[0, 0], reftrack[0, 1], 'go', markersize=10, label='Start/Finish', zorder=5)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('F1TENTH Racing Line (FIXED)')
    ax1.legend()
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Velocity profile
    if velocity_profile is not None:
        # Calculate distance along track
        distances = np.zeros(len(raceline))
        for i in range(1, len(raceline)):
            distances[i] = distances[i-1] + np.linalg.norm(raceline[i] - raceline[i-1])
        
        ax2.plot(distances, velocity_profile, 'g-', linewidth=2)
        ax2.fill_between(distances, 0, velocity_profile, alpha=0.3, color='green')
        ax2.set_xlabel('Distance (m)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity Profile')
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim(0, max(velocity_profile) * 1.1)
    
    # Plot 3: Curvature profile
    curvature = calculate_curvature_smooth(raceline)
    distances = np.zeros(len(raceline))
    for i in range(1, len(raceline)):
        distances[i] = distances[i-1] + np.linalg.norm(raceline[i] - raceline[i-1])
    
    ax3.plot(distances, curvature, 'purple', linewidth=2)
    ax3.fill_between(distances, 0, curvature, alpha=0.3, color='purple')
    ax3.set_xlabel('Distance (m)')
    ax3.set_ylabel('Curvature (1/m)')
    ax3.set_title('Curvature Profile')
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    
    # Plot 4: Lateral offset
    normals = calculate_track_normals_fixed(reftrack)
    lateral_offset = np.zeros(len(raceline))
    
    for i in range(len(raceline)):
        offset_vec = raceline[i] - reftrack[i, :2]
        lateral_offset[i] = np.dot(offset_vec, normals[i])
    
    ax4.plot(distances, lateral_offset, 'orange', linewidth=2, label='Raceline Position')
    ax4.fill_between(distances, -reftrack[:, 2], reftrack[:, 3], alpha=0.2, color='gray', label='Track Width')
    ax4.set_xlabel('Distance (m)')
    ax4.set_ylabel('Lateral Offset (m)')
    ax4.set_title('Racing Line Position (+ = left of centerline)')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color='k', linestyle='-', alpha=0.5, label='Centerline')
    ax4.legend()
    
    plt.tight_layout()
    
    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Clean visualization saved: {save_path}")
    
    if show_plot:
        plt.show()
    else:
        plt.close()

def main():
    print("F1TENTH Trajectory Optimization Tool (FIXED VERSION)")
    print("===================================================")
    
    t_start = time.perf_counter()
    
    # File path setup
    if any([args.centerline_path, args.output_path]):
        centerline_path = args.centerline_path
        output_path = args.output_path
        
        if not centerline_path:
            centerline_dir = SCRIPT_DIR / "centerline_with_boundary"
            potential_files = list(centerline_dir.glob("*.csv"))
            if potential_files:
                centerline_path = str(potential_files[0])
                print(f"Using default centerline file: {centerline_path}")
            else:
                print("ERROR: No centerline file found.")
                return
                
        if not output_path:
            centerline_name = Path(centerline_path).stem
            output_dir = SCRIPT_DIR / "raceline"
            output_dir.mkdir(exist_ok=True)
            output_path = str(output_dir / f"{centerline_name}_raceline_fixed.csv")
            
    else:
        print("팝업 다이얼로그로 파일을 선택하세요...")
        
        root = Tk()
        root.withdraw()
        
        centerline_path = select_centerline_file(None)
        centerline_name = Path(centerline_path).stem
        
        output_path = asksaveasfilename(
            title='Raceline CSV 파일 저장 위치 선택',
            filetypes=[('CSV Files', '*.csv'), ('All Files', '*.*')],
            initialdir='/home/f1/f1tenth_ws/path_generate/raceline',
            initialfile=f"{centerline_name}_raceline_fixed.csv",
            defaultextension='.csv'
        )
        if not output_path:
            sys.exit("출력 파일 경로가 선택되지 않았습니다.")
            
        root.destroy()
    
    # Load and process
    centerline_file = Path(centerline_path)
    if not centerline_file.exists():
        print(f"ERROR: Centerline file not found: {centerline_file}")
        return
    
    print(f"Loading centerline from: {centerline_file}")
    reftrack = load_centerline_with_boundaries(str(centerline_file))
    
    if reftrack is None:
        print("ERROR: Failed to load centerline data")
        return
    
    # Run FIXED optimization
    print("Running FIXED racing line optimization...")
    alpha_opt = racing_line_optimization(reftrack, optim_opts["safety_margin"])
    
    # Generate results
    raceline = create_raceline(reftrack, alpha_opt)
    kappa = calculate_curvature_smooth(raceline)  # Use smooth curvature
    velocity_profile = calculate_velocity_profile(raceline, kappa, veh_params["v_max"])
    
    # Print file information
    print(f"\n파일 정보:")
    print(f"  Input: {centerline_path}")
    print(f"  Output: {output_path}")
    
    # Export results
    output_file = Path(output_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    
    # Export main raceline (x,y format)
    export_raceline_csv(raceline, velocity_profile, str(output_file), 'simple')
    
    # Export additional formats
    output_stem = output_file.stem
    output_dir = output_file.parent
    
    raceline_vel_file = output_dir / f"{output_stem}_with_velocity.csv" 
    export_raceline_csv(raceline, velocity_profile, str(raceline_vel_file), 'with_velocity')
    
    raceline_full_file = output_dir / f"{output_stem}_full.csv"
    export_raceline_csv(raceline, velocity_profile, str(raceline_full_file), 'full')
    
    # Generate CLEAN visualization
    plot_path = output_dir / f"{output_stem}_visualization.png"
    plot_results_clean(reftrack, raceline, velocity_profile, str(plot_path), args.show_plot)
    
    # Print results
    track_length = np.sum([np.linalg.norm(raceline[i+1] - raceline[i]) for i in range(len(raceline)-1)])
    avg_velocity = np.mean(velocity_profile)
    estimated_laptime = track_length / avg_velocity if avg_velocity > 0 else 0
    max_curvature = np.max(np.abs(kappa))
    
    print(f"\n=== RESULTS ===")
    print(f"Track length: {track_length:.1f}m")
    print(f"Average speed: {avg_velocity:.1f}m/s") 
    print(f"Max speed: {np.max(velocity_profile):.1f}m/s")
    print(f"Max curvature: {max_curvature:.3f} 1/m")
    print(f"Estimated lap time: {estimated_laptime:.1f}s")
    print(f"Runtime: {time.perf_counter() - t_start:.2f}s")
    print(f"Method: {'Scipy optimization' if SCIPY_AVAILABLE else 'Geometric optimization'}")
    print("=================")

if __name__ == "__main__":
    main()