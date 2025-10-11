#include "map_controller/steering_lookup.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace map_controller
{

namespace
{
std::string findLookupPath(const std::string & lut_name)
{
  const std::string pkg_share = ament_index_cpp::get_package_share_directory("map_controller");
  return pkg_share + "/config/steering_lookup/" + lut_name + "_lookup_table.csv";
}

std::vector<double> parseRow(const std::string & line)
{
  std::vector<double> row;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    try {
      row.push_back(std::stod(token));
    } catch (const std::exception &) {
      row.push_back(std::numeric_limits<double>::quiet_NaN());
    }
  }
  return row;
}
}  // namespace

SteeringLookup::SteeringLookup(const std::string & lut_name)
{
  const auto csv_path = findLookupPath(lut_name);
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open steering lookup table: " + csv_path);
  }

  std::string line;
  bool header_processed = false;
  while (std::getline(file, line)) {
    const auto row = parseRow(line);
    if (!header_processed) {
      velocities_.assign(row.begin() + 1, row.end());
      header_processed = true;
      continue;
    }

    if (row.empty()) {
      continue;
    }

    steer_angles_.push_back(row.front());
    std::vector<double> values(row.begin() + 1, row.end());
    table_.push_back(std::move(values));
  }
}

std::optional<double> SteeringLookup::lookup(double lateral_accel, double velocity_mps) const
{
  if (velocities_.empty() || steer_angles_.empty() || table_.empty()) {
    return std::nullopt;
  }

  const double sign = lateral_accel >= 0.0 ? 1.0 : -1.0;
  const double accel = std::fabs(lateral_accel);
  const double velocity = std::max(velocities_.front(), std::min(velocity_mps, velocities_.back()));

  auto upper_vel_it = std::lower_bound(velocities_.begin(), velocities_.end(), velocity);
  std::size_t upper_vel_idx = static_cast<std::size_t>(std::distance(velocities_.begin(), upper_vel_it));
  if (upper_vel_idx == 0) {
    upper_vel_idx = 1;
  }
  if (upper_vel_idx >= velocities_.size()) {
    upper_vel_idx = velocities_.size() - 1;
  }
  const std::size_t lower_vel_idx = upper_vel_idx - 1;

  const double v0 = velocities_[lower_vel_idx];
  const double v1 = velocities_[upper_vel_idx];
  const double vel_ratio = (v1 - v0) > 1e-6 ? (velocity - v0) / (v1 - v0) : 0.0;

  auto interpolate_for_velocity = [&](std::size_t column_idx) -> double {
    std::vector<double> accel_column(table_.size());
    for (std::size_t row = 0; row < table_.size(); ++row) {
      accel_column[row] = table_[row][column_idx];
    }

    auto upper_acc_it = std::lower_bound(accel_column.begin(), accel_column.end(), accel);
    std::size_t upper_acc_idx = static_cast<std::size_t>(std::distance(accel_column.begin(), upper_acc_it));
    if (upper_acc_idx == 0) {
      upper_acc_idx = 1;
    }
    if (upper_acc_idx >= accel_column.size()) {
      upper_acc_idx = accel_column.size() - 1;
    }
    const std::size_t lower_acc_idx = upper_acc_idx - 1;

    const double a0 = accel_column[lower_acc_idx];
    const double a1 = accel_column[upper_acc_idx];
    const double s0 = steer_angles_[lower_acc_idx];
    const double s1 = steer_angles_[upper_acc_idx];
    const double ratio = (a1 - a0) > 1e-6 ? (accel - a0) / (a1 - a0) : 0.0;
    return s0 + (s1 - s0) * ratio;
  };

  const double steer0 = interpolate_for_velocity(lower_vel_idx);
  const double steer1 = interpolate_for_velocity(upper_vel_idx);
  const double steering = steer0 + (steer1 - steer0) * vel_ratio;
  return sign * steering;
}

}  // namespace map_controller
