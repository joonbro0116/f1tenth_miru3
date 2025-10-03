#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <cstdint>

namespace map_controller
{
class SteeringLookup;

struct Waypoint
{
  double x_m{};
  double y_m{};
  double target_speed_mps{};
  double frenet_d{};
  double frenet_s{};
  double curvature_radpm{};
  double heading_rad{};
  double longitudinal_accel_mps2{};
};

struct VehicleState
{
  double x_m{};
  double y_m{};
  double yaw_rad{};
  double speed_mps{};
  double acceleration_mps2{};
  double frenet_s_m{};
  double frenet_d_m{};
  double frenet_speed_mps{};
};

struct ControllerOutput
{
  double target_speed_mps{};
  double target_accel_mps2{};
  double steering_angle_rad{};
  double l1_distance_m{};
  std::array<double, 2> lookahead_point{{0.0, 0.0}};
};

struct OpponentState
{
  double frenet_s_m{};
  double frenet_d_m{};
  double speed_mps{};
  double acceleration_mps2{};
  bool is_visible{false};
};

enum class DrivingMode
{
  Racing,
  Trailing
};

struct MapControllerParams
{
  double t_clip_min{0.8};
  double t_clip_max{5.0};
  double m_l1{0.6};
  double q_l1{-0.18};
  double speed_lookahead_s{0.25};
  double lat_err_coeff{1.0};
  double acc_scaler_for_steer{1.2};
  double dec_scaler_for_steer{0.9};
  double start_scale_speed_mps{7.0};
  double end_scale_speed_mps{8.0};
  double downscale_factor{0.2};
  double speed_lookahead_for_steer_s{0.0};
  bool prioritize_dyn{false};
  double trailing_gap_m{2.0};
  double trailing_p_gain{0.0};
  double trailing_i_gain{0.0};
  double trailing_d_gain{0.0};
  double blind_trailing_speed_mps{0.0};
  double loop_rate_hz{20.0};
  std::string steering_lut_name{"default"};
  double state_machine_rate_hz{20.0};
  double steering_change_threshold_rad{0.4};
};

class MapController
{
public:
  explicit MapController(const MapControllerParams & params);
  ~MapController();

  ControllerOutput computeCommand(const VehicleState & state,
                                  const std::vector<Waypoint> & local_path);

  void setDrivingMode(DrivingMode mode);
  void setOpponentState(const OpponentState & opponent);
  void clearOpponent();
  void setTrackLength(double track_length_m);

private:
  struct LateralErrorResult
  {
    double normalized{};
    double absolute{};
  };

  struct L1Result
  {
    std::array<double, 2> point_{0.0, 0.0};
    double distance_m{0.0};
  };

  std::size_t findNearestWaypointIndex(const VehicleState & state,
                                       const std::vector<Waypoint> & local_path) const;
  double computeAverageCurvatureAhead(const std::vector<Waypoint> & local_path,
                                      std::size_t start_index,
                                      std::size_t sample_count) const;
  LateralErrorResult computeLateralError(const VehicleState & state) const;
  double adjustSpeedForLateralError(double target_speed_mps,
                                    double lat_error_norm,
                                    double curvature_metric) const;
  double adjustSpeedForHeading(double target_speed_mps,
                               const VehicleState & state,
                               const std::vector<Waypoint> & local_path) const;
  double computeSpeedCommand(const VehicleState & state,
                             const std::vector<Waypoint> & local_path,
                             double lat_error_norm);
  double computeTrailingCommand(const VehicleState & state,
                                double global_speed_mps);
  std::size_t findNearestWaypointIndex(const std::array<double, 2> & position,
                                       const std::vector<Waypoint> & local_path) const;
  L1Result computeL1Point(const VehicleState & state,
                          const std::vector<Waypoint> & local_path,
                          double lateral_error_abs) const;
  double computeSteeringAngle(const VehicleState & state,
                              const std::vector<Waypoint> & local_path,
                              const L1Result & l1,
                              double lat_error_norm,
                              double desired_speed_mps);
  std::array<double, 2> propagatePosition(const VehicleState & state,
                                          double lookahead_time_s) const;
  double speedAdjustLatErr(double global_speed_mps,
                           double lat_error_norm,
                           double curvature_metric) const;
  double scaleSteeringForAcceleration(double steering_angle_rad,
                                      double acceleration_mps2) const;
  double scaleSteeringForSpeed(double steering_angle_rad,
                               double speed_mps) const;
  double clamp(double value, double min_value, double max_value) const;

  MapControllerParams params_;
  DrivingMode driving_mode_{DrivingMode::Racing};
  std::optional<OpponentState> opponent_state_;
  double track_length_m_{0.0};

  std::deque<double> lateral_error_history_;
  std::size_t nearest_waypoint_index_{0};
  double average_curvature_ahead_{0.0};
  double current_steering_angle_rad_{0.0};
  double trailing_integral_{0.0};
  double trailing_command_mps_{0.0};
  std::unique_ptr<SteeringLookup> steering_lookup_;
};
}  // namespace map_controller
