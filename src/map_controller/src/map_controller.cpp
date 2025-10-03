#include "map_controller/map_controller.hpp"
#include "map_controller/steering_lookup.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace map_controller
{
MapController::MapController(const MapControllerParams & params)
: params_(params),
  steering_lookup_(std::make_unique<SteeringLookup>(params_.steering_lut_name))
{
}

MapController::~MapController() = default;

void MapController::setDrivingMode(DrivingMode mode)
{
  driving_mode_ = mode;
}

void MapController::setOpponentState(const OpponentState & opponent)
{
  opponent_state_ = opponent;
}

void MapController::clearOpponent()
{
  opponent_state_.reset();
}

void MapController::setTrackLength(double track_length_m)
{
  track_length_m_ = std::max(0.0, track_length_m);
}

ControllerOutput MapController::computeCommand(const VehicleState & state,
                                               const std::vector<Waypoint> & local_path)
{
  ControllerOutput out{};

  if (local_path.empty()) {
    return out;
  }

  nearest_waypoint_index_ = findNearestWaypointIndex(state, local_path);
  average_curvature_ahead_ = computeAverageCurvatureAhead(local_path, nearest_waypoint_index_, 25);

  const auto lateral_error = computeLateralError(state);

  if (lateral_error_history_.size() > 50U) {
    lateral_error_history_.pop_front();
  }
  lateral_error_history_.push_back(lateral_error.absolute);

  double target_speed_mps = computeSpeedCommand(state, local_path, lateral_error.normalized);

  const auto l1 = computeL1Point(state, local_path, lateral_error.absolute);
  const double steering_angle = computeSteeringAngle(state, local_path, l1, lateral_error.normalized, target_speed_mps);

  out.target_speed_mps = target_speed_mps;
  out.target_accel_mps2 = local_path[nearest_waypoint_index_].longitudinal_accel_mps2;
  out.steering_angle_rad = steering_angle;
  out.l1_distance_m = l1.distance_m;
  out.lookahead_point = l1.point_;

  return out;
}

std::size_t MapController::findNearestWaypointIndex(const VehicleState & state,
                                                    const std::vector<Waypoint> & local_path) const
{
  double best_distance_sq = std::numeric_limits<double>::max();
  std::size_t best_index = 0;

  for (std::size_t i = 0; i < local_path.size(); ++i) {
    const double dx = state.x_m - local_path[i].x_m;
    const double dy = state.y_m - local_path[i].y_m;
    const double distance_sq = dx * dx + dy * dy;

    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_index = i;
    }
  }

  return best_index;
}

std::size_t MapController::findNearestWaypointIndex(const std::array<double, 2> & position,
                                                    const std::vector<Waypoint> & local_path) const
{
  double best_distance_sq = std::numeric_limits<double>::max();
  std::size_t best_index = 0;

  for (std::size_t i = 0; i < local_path.size(); ++i) {
    const double dx = position[0] - local_path[i].x_m;
    const double dy = position[1] - local_path[i].y_m;
    const double distance_sq = dx * dx + dy * dy;

    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_index = i;
    }
  }

  return best_index;
}

double MapController::computeAverageCurvatureAhead(const std::vector<Waypoint> & local_path,
                                                   std::size_t start_index,
                                                   std::size_t sample_count) const
{
  if (local_path.empty()) {
    return 0.0;
  }

  const std::size_t max_index = std::min<std::size_t>(local_path.size(), start_index + sample_count);
  double sum_curvature = 0.0;
  std::size_t counted = 0;

  for (std::size_t i = start_index; i < max_index; ++i) {
    sum_curvature += std::abs(local_path[i].curvature_radpm);
    ++counted;
  }

  if (counted == 0) {
    return 0.0;
  }

  return sum_curvature / static_cast<double>(counted);
}

double MapController::computeSpeedCommand(const VehicleState & state,
                                          const std::vector<Waypoint> & local_path,
                                          double lat_error_norm)
{
  const auto propagated_position = propagatePosition(state, params_.speed_lookahead_s);
  const std::size_t lookahead_index = findNearestWaypointIndex(propagated_position, local_path);
  const double global_speed = local_path[lookahead_index].target_speed_mps;

  double speed_command = global_speed;

  if (driving_mode_ == DrivingMode::Trailing && opponent_state_) {
    speed_command = computeTrailingCommand(state, global_speed);
  } else {
    trailing_integral_ = 0.0;
    trailing_command_mps_ = global_speed;
  }

  speed_command = adjustSpeedForLateralError(speed_command, lat_error_norm, average_curvature_ahead_);
  speed_command = adjustSpeedForHeading(speed_command, state, local_path);

  return std::max(0.0, speed_command);
}

double MapController::computeTrailingCommand(const VehicleState & state, double global_speed_mps)
{
  if (!opponent_state_ || track_length_m_ <= 0.0 || params_.loop_rate_hz <= 0.0) {
    trailing_command_mps_ = global_speed_mps;
    return trailing_command_mps_;
  }

  const auto & opponent = opponent_state_.value();

  double gap = opponent.frenet_s_m - state.frenet_s_m;
  if (gap < 0.0) {
    gap += track_length_m_;
  }
  gap = std::fmod(gap, track_length_m_);

  const double gap_should = params_.trailing_gap_m;
  const double gap_error = gap_should - gap;
  const double velocity_diff = state.frenet_speed_mps - opponent.speed_mps;

  trailing_integral_ += gap_error / params_.loop_rate_hz;
  trailing_integral_ = clamp(trailing_integral_, -10.0, 10.0);

  const double p_value = gap_error * params_.trailing_p_gain;
  const double i_value = trailing_integral_ * params_.trailing_i_gain;
  const double d_value = velocity_diff * params_.trailing_d_gain;

  trailing_command_mps_ = opponent.speed_mps - p_value - i_value - d_value;
  trailing_command_mps_ = clamp(trailing_command_mps_, 0.0, global_speed_mps);

  if (!opponent.is_visible && gap > gap_should) {
    trailing_command_mps_ = std::max(params_.blind_trailing_speed_mps, trailing_command_mps_);
  }

  return trailing_command_mps_;
}

MapController::LateralErrorResult MapController::computeLateralError(const VehicleState & state) const
{
  LateralErrorResult result{};

  const double lateral_error = std::abs(state.frenet_d_m);
  result.absolute = lateral_error;

  constexpr double max_lat_error = 0.5;
  constexpr double min_lat_error = 0.0;
  const double clipped = clamp(lateral_error, min_lat_error, max_lat_error);

  if (max_lat_error > min_lat_error) {
    result.normalized = 0.5 * ((clipped - min_lat_error) / (max_lat_error - min_lat_error));
  } else {
    result.normalized = 0.0;
  }

  return result;
}

double MapController::adjustSpeedForLateralError(double target_speed_mps,
                                                 double lat_error_norm,
                                                 double curvature_metric) const
{
  const double lat_coeff = clamp(params_.lat_err_coeff, 0.0, 1.0);
  const double scaled_lat_norm = lat_error_norm * 2.0;
  const double curvature_component = clamp(2.0 * (curvature_metric / 0.8) - 2.0, 0.0, 1.0);

  const double scaling = (1.0 - lat_coeff) + lat_coeff * std::exp(-scaled_lat_norm * curvature_component);
  return target_speed_mps * scaling;
}

double MapController::adjustSpeedForHeading(double target_speed_mps,
                                            const VehicleState & state,
                                            const std::vector<Waypoint> & local_path) const
{
  if (local_path.empty()) {
    return target_speed_mps;
  }

  const double heading = state.yaw_rad;
  const double map_heading = local_path[nearest_waypoint_index_].heading_rad;

  double heading_error = std::abs(heading - map_heading);
  constexpr double kPi = 3.14159265358979323846;
  if (heading_error > kPi) {
    heading_error = 2.0 * kPi - heading_error;
  }

  const double threshold_ok = kPi / 9.0;
  if (heading_error < threshold_ok) {
    return target_speed_mps;
  }

  double scaler = 0.5;
  const double ninety_deg = kPi / 2.0;
  if (heading_error < ninety_deg) {
    scaler = 1.0 - 0.5 * heading_error / ninety_deg;
  }

  return target_speed_mps * scaler;
}

MapController::L1Result MapController::computeL1Point(const VehicleState & state,
                                                      const std::vector<Waypoint> & local_path,
                                                      double lateral_error_abs) const
{
  L1Result result{};

  if (local_path.empty()) {
    return result;
  }

  double l1_distance = params_.q_l1 + state.speed_mps * params_.m_l1;
  const double lower_bound = std::max(params_.t_clip_min, std::sqrt(2.0) * lateral_error_abs);
  l1_distance = clamp(l1_distance, lower_bound, params_.t_clip_max);
  result.distance_m = l1_distance;

  double accumulated = 0.0;
  std::size_t index = nearest_waypoint_index_;

  while (index + 1 < local_path.size() && accumulated < l1_distance) {
    const double dx = local_path[index + 1].x_m - local_path[index].x_m;
    const double dy = local_path[index + 1].y_m - local_path[index].y_m;
    const double segment = std::hypot(dx, dy);
    accumulated += segment;
    ++index;
  }

  index = std::min(index, local_path.size() - 1);
  result.point_[0] = local_path[index].x_m;
  result.point_[1] = local_path[index].y_m;

  return result;
}

double MapController::computeSteeringAngle(const VehicleState & state,
                                           const std::vector<Waypoint> & local_path,
                                           const L1Result & l1,
                                           double lat_error_norm,
                                           double desired_speed_mps)
{
  if (local_path.empty()) {
    return 0.0;
  }

  double speed_for_lookup = desired_speed_mps;

  if (driving_mode_ == DrivingMode::Trailing && opponent_state_) {
    speed_for_lookup = state.speed_mps;
  } else {
    const auto propagated_for_steer = propagatePosition(state, params_.speed_lookahead_for_steer_s);
    const std::size_t idx_la = findNearestWaypointIndex(propagated_for_steer, local_path);
    const double global_speed = local_path[idx_la].target_speed_mps;
    speed_for_lookup = adjustSpeedForLateralError(global_speed, lat_error_norm, average_curvature_ahead_);
  }

  const std::array<double, 2> vehicle_position{state.x_m, state.y_m};
  const std::array<double, 2> l1_vector{l1.point_[0] - vehicle_position[0], l1.point_[1] - vehicle_position[1]};
  const double l1_norm = std::hypot(l1_vector[0], l1_vector[1]);

  double eta = 0.0;
  if (l1_norm > 1e-6) {
    const double sin_component = (-std::sin(state.yaw_rad)) * l1_vector[0] + (std::cos(state.yaw_rad)) * l1_vector[1];
    const double ratio = clamp(sin_component / l1_norm, -1.0, 1.0);
    eta = std::asin(ratio);
  }

  double lateral_acc = 0.0;
  if (l1.distance_m > 1e-6) {
    const double sin_eta = std::sin(eta);
    if (std::abs(sin_eta) > 1e-6) {
      lateral_acc = 2.0 * speed_for_lookup * speed_for_lookup / l1.distance_m * sin_eta;
    }
  }

  double steering_angle = steering_lookup_ ? steering_lookup_->lookup(lateral_acc, speed_for_lookup) : 0.0;
  steering_angle = scaleSteeringForAcceleration(steering_angle, state.acceleration_mps2);
  steering_angle = scaleSteeringForSpeed(steering_angle, speed_for_lookup);

  const double velocity_scaler = clamp(1.0 + (state.speed_mps / 10.0), 1.0, 1.25);
  steering_angle *= velocity_scaler;

  const double threshold = params_.steering_change_threshold_rad;
  const double min_angle = current_steering_angle_rad_ - threshold;
  const double max_angle = current_steering_angle_rad_ + threshold;
  const double clipped = clamp(steering_angle, min_angle, max_angle);

  current_steering_angle_rad_ = clipped;
  return clipped;
}

std::array<double, 2> MapController::propagatePosition(const VehicleState & state,
                                                       double lookahead_time_s) const
{
  const double dt = std::max(0.0, lookahead_time_s);
  const double distance = state.speed_mps * dt;
  const double dx = std::cos(state.yaw_rad) * distance;
  const double dy = std::sin(state.yaw_rad) * distance;
  return {state.x_m + dx, state.y_m + dy};
}

double MapController::speedAdjustLatErr(double global_speed_mps,
                                        double lat_error_norm,
                                        double curvature_metric) const
{
  return adjustSpeedForLateralError(global_speed_mps, lat_error_norm, curvature_metric);
}

double MapController::scaleSteeringForAcceleration(double steering_angle_rad,
                                                   double acceleration_mps2) const
{
  if (acceleration_mps2 >= 1.0) {
    return steering_angle_rad * params_.acc_scaler_for_steer;
  }

  if (acceleration_mps2 <= -1.0) {
    return steering_angle_rad * params_.dec_scaler_for_steer;
  }

  return steering_angle_rad;
}

double MapController::scaleSteeringForSpeed(double steering_angle_rad,
                                            double speed_mps) const
{
  const double speed_diff = std::max(0.1, params_.end_scale_speed_mps - params_.start_scale_speed_mps);
  const double factor = 1.0 - clamp((speed_mps - params_.start_scale_speed_mps) / speed_diff, 0.0, 1.0) * params_.downscale_factor;
  return steering_angle_rad * factor;
}

double MapController::clamp(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}
}  // namespace map_controller
