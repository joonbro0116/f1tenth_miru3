#include "map_controller/map_controller.hpp"

#include "map_controller/steering_lookup.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace map_controller
{

namespace
{
double distance2d(double x0, double y0, double x1, double y1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  return std::sqrt(dx * dx + dy * dy);
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}
}

MapController::MapController(const MapControllerParams & params)
: params_(params)
{
  try {
    steering_lookup_ = std::make_unique<SteeringLookup>(params_.steering_lut_name);
  } catch (const std::exception &) {
    steering_lookup_.reset();
  }
}

ControllerOutput MapController::computeCommand(const VehicleState & state,
                                               const std::vector<Waypoint> & path)
{
  ControllerOutput output{};
  if (path.size() < 2) {
    return output;
  }

  const std::size_t nearest_idx = findNearestWaypointIndex(state, path);
  const auto lateral_error = computeLateralError(state, path, nearest_idx);
  const auto l1_result = computeL1Point(state, path, nearest_idx, lateral_error.absolute);
  double desired_speed = computeSpeedCommand(state, path, nearest_idx, lateral_error.normalized);

  output.target_speed_mps = desired_speed;
  output.target_accel_mps2 = 0.0;
  output.l1_distance_m = l1_result.distance;
  output.lookahead_point = l1_result.point;
  output.steering_angle_rad = computeSteeringAngle(state, l1_result, desired_speed, lateral_error.normalized);

  previous_steering_ = output.steering_angle_rad;
  return output;
}

std::size_t MapController::findNearestWaypointIndex(const VehicleState & state,
                                                    const std::vector<Waypoint> & path) const
{
  double min_distance = std::numeric_limits<double>::max();
  std::size_t index = 0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double dist = distance2d(state.x_m, state.y_m, path[i].x_m, path[i].y_m);
    if (dist < min_distance) {
      min_distance = dist;
      index = i;
    }
  }
  return index;
}

MapController::LateralError MapController::computeLateralError(const VehicleState & state,
                                                               const std::vector<Waypoint> & path,
                                                               std::size_t nearest_idx) const
{
  const std::size_t next_idx = (nearest_idx + 1) % path.size();
  const double path_dx = path[next_idx].x_m - path[nearest_idx].x_m;
  const double path_dy = path[next_idx].y_m - path[nearest_idx].y_m;
  const double path_heading = std::atan2(path_dy, path_dx);

  const double dx = state.x_m - path[nearest_idx].x_m;
  const double dy = state.y_m - path[nearest_idx].y_m;
  const double cross = path_dx * dy - path_dy * dx;
  const double lat_error = (cross >= 0.0 ? 1.0 : -1.0) * std::sqrt(dx * dx + dy * dy);

  LateralError result;
  result.absolute = std::fabs(lat_error);
  result.sign = cross >= 0.0 ? 1 : -1;
  const double heading_error = normalizeAngle(state.yaw_rad - path_heading);
  const double norm_denom = std::max(params_.t_clip_max, 1.0);
  result.normalized = std::min(1.0, result.absolute / norm_denom + std::fabs(heading_error) * 0.1);
  return result;
}

MapController::L1Result MapController::computeL1Point(const VehicleState & state,
                                                      const std::vector<Waypoint> & path,
                                                      std::size_t nearest_idx,
                                                      double lateral_error_abs) const
{
  const double desired_distance = std::clamp(params_.q_l1 + params_.m_l1 * state.speed_mps,
                                             params_.t_clip_min,
                                             params_.t_clip_max);
  const double lower_bound = std::max(params_.t_clip_min, std::sqrt(2.0) * lateral_error_abs);
  const double l1_distance = std::max(desired_distance, lower_bound);

  double accumulated = 0.0;
  std::size_t idx = nearest_idx;
  while (accumulated < l1_distance && idx + 1 < path.size()) {
    const double segment = distance2d(path[idx].x_m, path[idx].y_m,
                                      path[idx + 1].x_m, path[idx + 1].y_m);
    accumulated += segment;
    ++idx;
  }
  if (idx >= path.size()) {
    idx = path.size() - 1;
  }

  L1Result result;
  result.distance = l1_distance;
  result.point = {path[idx].x_m, path[idx].y_m};
  return result;
}

double MapController::computeSpeedCommand(const VehicleState & state,
                                          const std::vector<Waypoint> & path,
                                          std::size_t nearest_idx,
                                          double lat_error_norm) const
{
  const std::size_t lookahead_idx = std::min(path.size() - 1,
                                             nearest_idx + static_cast<std::size_t>(params_.speed_lookahead_s * 10.0));
  double target_speed = path[lookahead_idx].target_speed_mps;
  const double reduction = params_.lat_err_coeff * lat_error_norm;
  target_speed *= std::max(0.0, 1.0 - reduction);
  target_speed = std::max(0.0, target_speed);

  const double max_delta = 1.5;  // m/s per cycle
  const double delta = target_speed - state.speed_mps;
  if (delta > max_delta) {
    target_speed = state.speed_mps + max_delta;
  } else if (delta < -max_delta) {
    target_speed = state.speed_mps - max_delta;
  }

  return target_speed;
}

double MapController::computeSteeringAngle(const VehicleState & state,
                                           const L1Result & l1,
                                           double desired_speed_mps,
                                           double lat_error_norm)
{
  const double dx = l1.point[0] - state.x_m;
  const double dy = l1.point[1] - state.y_m;
  const double distance = std::max(0.001, std::sqrt(dx * dx + dy * dy));
  const double heading_to_point = std::atan2(dy, dx);
  const double eta = normalizeAngle(heading_to_point - state.yaw_rad);

  const double lat_acc = 2.0 * desired_speed_mps * desired_speed_mps * std::sin(eta) / distance;
  double steering = 0.0;
  if (steering_lookup_) {
    const auto value = steering_lookup_->lookup(lat_acc, desired_speed_mps);
    steering = value.value_or(std::atan2(2.0 * std::sin(eta), distance));
  } else {
    steering = std::atan2(2.0 * std::sin(eta), distance);
  }

  // Speed based scaling similar to MAP
  if (desired_speed_mps > params_.start_scale_speed_mps) {
    const double clamped_speed = std::min(desired_speed_mps, params_.end_scale_speed_mps);
    const double factor = 1.0 - params_.downscale_factor *
      (clamped_speed - params_.start_scale_speed_mps) /
      std::max(0.001, (params_.end_scale_speed_mps - params_.start_scale_speed_mps));
    steering *= std::clamp(factor, 1.0 - params_.downscale_factor, 1.0);
  }

  // limit steering change
  const double delta = steering - previous_steering_;
  const double max_delta = params_.steering_change_threshold_rad;
  if (delta > max_delta) {
    steering = previous_steering_ + max_delta;
  } else if (delta < -max_delta) {
    steering = previous_steering_ - max_delta;
  }

  return steering;
}

double MapController::clamp(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

double MapController::wrapAngle(double angle)
{
  return normalizeAngle(angle);
}

}  // namespace map_controller
