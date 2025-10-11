#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "map_controller/steering_lookup.hpp"

namespace map_controller
{

struct Waypoint
{
  double x_m{0.0};
  double y_m{0.0};
  double target_speed_mps{0.0};
  double heading_rad{0.0};
  double curvature{0.0};
};

struct VehicleState
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
};

struct ControllerOutput
{
  double target_speed_mps{0.0};
  double target_accel_mps2{0.0};
  double steering_angle_rad{0.0};
  double l1_distance_m{0.0};
  std::array<double, 2> lookahead_point{0.0, 0.0};
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
  double steering_change_threshold_rad{0.4};
  std::string steering_lut_name{"NUC2_pacejka"};
};

class MapController
{
public:
  explicit MapController(const MapControllerParams & params);

  ControllerOutput computeCommand(const VehicleState & state,
                                  const std::vector<Waypoint> & path);

private:
  struct LateralError
  {
    double absolute{0.0};
    double normalized{0.0};
    int sign{1};
  };

  struct L1Result
  {
    std::array<double, 2> point{0.0, 0.0};
    double distance{0.0};
  };

  std::size_t findNearestWaypointIndex(const VehicleState & state,
                                       const std::vector<Waypoint> & path) const;
  LateralError computeLateralError(const VehicleState & state,
                                   const std::vector<Waypoint> & path,
                                   std::size_t nearest_idx) const;
  L1Result computeL1Point(const VehicleState & state,
                          const std::vector<Waypoint> & path,
                          std::size_t nearest_idx,
                          double lateral_error_abs) const;
  double computeSpeedCommand(const VehicleState & state,
                             const std::vector<Waypoint> & path,
                             std::size_t nearest_idx,
                             double lat_error_norm) const;
  double computeSteeringAngle(const VehicleState & state,
                              const L1Result & l1,
                              double desired_speed_mps,
                              double lat_error_norm);
  double clamp(double value, double min_value, double max_value) const;
  static double wrapAngle(double angle);

  MapControllerParams params_;
  std::unique_ptr<SteeringLookup> steering_lookup_;
  mutable double previous_steering_{0.0};
};

}  // namespace map_controller
