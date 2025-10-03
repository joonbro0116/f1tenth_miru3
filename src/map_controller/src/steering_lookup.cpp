#include "map_controller/steering_lookup.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace map_controller
{
namespace
{
constexpr double kEpsilon = 1e-9;
}

SteeringLookup::SteeringLookup(const std::string & table_name)
: table_name_(table_name)
{
  loadTable(table_name);
}

double SteeringLookup::lookup(double lateral_accel, double longitudinal_speed) const
{
  if (speed_axis_.empty() || steering_axis_.empty() || accel_table_.empty()) {
    return 0.0;
  }

  const double sign = (lateral_accel >= 0.0) ? 1.0 : -1.0;
  const double accel = std::abs(lateral_accel);

  const std::size_t speed_index = findNearestIndex(speed_axis_, longitudinal_speed);

  std::vector<double> column;
  column.reserve(accel_table_.size());
  for (const auto & row : accel_table_) {
    if (speed_index < row.size()) {
      column.push_back(row[speed_index]);
    } else {
      column.push_back(std::numeric_limits<double>::quiet_NaN());
    }
  }

  const auto neighbors = findClosestNeighbors(column, accel);

  double steering = 0.0;
  if (neighbors.primary_index >= steering_axis_.size()) {
    return 0.0;
  }

  const double primary_steer = steering_axis_[neighbors.primary_index];
  const double secondary_steer = (neighbors.secondary_index < steering_axis_.size())
                                   ? steering_axis_[neighbors.secondary_index]
                                   : primary_steer;

  if (neighbors.primary_index == neighbors.secondary_index ||
      std::abs(neighbors.secondary_value - neighbors.primary_value) < kEpsilon) {
    steering = primary_steer;
  } else {
    const double ratio = (accel - neighbors.primary_value) /
                         (neighbors.secondary_value - neighbors.primary_value);
    steering = primary_steer + ratio * (secondary_steer - primary_steer);
  }

  return steering * sign;
}

void SteeringLookup::loadTable(const std::string & table_name)
{
  const auto share_dir = ament_index_cpp::get_package_share_directory("map_controller");
  const std::filesystem::path file_path =
    std::filesystem::path(share_dir) / "resources" / "lut" /
    (table_name + "_lookup_table.csv");

  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open steering lookup table: " + file_path.string());
  }

  std::string line;
  bool first_line = true;

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::vector<double> row_values;
    std::stringstream ss(line);
    std::string cell;

    while (std::getline(ss, cell, ',')) {
      try {
        row_values.push_back(std::stod(cell));
      } catch (const std::invalid_argument &) {
        row_values.push_back(std::numeric_limits<double>::quiet_NaN());
      }
    }

    if (row_values.empty()) {
      continue;
    }

    if (first_line) {
      if (row_values.size() < 2) {
        throw std::runtime_error("Steering lookup table header must contain at least two columns");
      }
      speed_axis_.assign(row_values.begin() + 1, row_values.end());
      first_line = false;
      continue;
    }

    steering_axis_.push_back(row_values.front());
    row_values.erase(row_values.begin());
    accel_table_.push_back(std::move(row_values));
  }

  if (speed_axis_.empty() || steering_axis_.empty() || accel_table_.empty()) {
    throw std::runtime_error("Steering lookup table is empty or malformed: " + file_path.string());
  }
}

std::size_t SteeringLookup::findNearestIndex(const std::vector<double> & axis, double value) const
{
  if (axis.empty()) {
    return 0;
  }

  double best_diff = std::numeric_limits<double>::max();
  std::size_t best_index = 0;

  for (std::size_t i = 0; i < axis.size(); ++i) {
    const double diff = std::abs(axis[i] - value);
    if (diff < best_diff) {
      best_diff = diff;
      best_index = i;
    }
  }

  return best_index;
}

SteeringLookup::NeighborPair SteeringLookup::findClosestNeighbors(const std::vector<double> & values,
                                                                  double target) const
{
  NeighborPair result{};

  if (values.empty()) {
    return result;
  }

  std::vector<std::pair<double, std::size_t>> valid_values;
  valid_values.reserve(values.size());

  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isnan(values[i])) {
      valid_values.emplace_back(values[i], i);
    }
  }

  if (valid_values.empty()) {
    return result;
  }

  double best_diff = std::numeric_limits<double>::max();
  std::size_t best_index = 0;

  for (std::size_t i = 0; i < valid_values.size(); ++i) {
    const double diff = std::abs(valid_values[i].first - target);
    if (diff < best_diff) {
      best_diff = diff;
      best_index = i;
    }
  }

  const auto select_neighbor = [&](std::size_t idx) {
    if (idx <= 0 || idx >= valid_values.size() - 1) {
      return idx;
    }

    const double diff_prev = std::abs(valid_values[idx - 1].first - target);
    const double diff_next = std::abs(valid_values[idx + 1].first - target);
    return (diff_prev <= diff_next) ? idx - 1 : idx + 1;
  };

  const std::size_t neighbor_index = select_neighbor(best_index);

  result.primary_value = valid_values[best_index].first;
  result.primary_index = valid_values[best_index].second;
  result.secondary_value = valid_values[neighbor_index].first;
  result.secondary_index = valid_values[neighbor_index].second;

  return result;
}
}  // namespace map_controller
