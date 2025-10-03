#pragma once

#include <vector>
#include <cstdint>

namespace map_controller
{
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
};

struct ControllerOutput
{
  double target_speed_mps{};
  double target_accel_mps2{};
  double steering_angle_rad{};
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
};

class MapController
{
public:
  explicit MapController(const MapControllerParams & params);

  ControllerOutput computeCommand(const VehicleState & state,
                                  const std::vector<Waypoint> & local_path);

private:
  MapControllerParams params_;
};
}  // namespace map_controller
