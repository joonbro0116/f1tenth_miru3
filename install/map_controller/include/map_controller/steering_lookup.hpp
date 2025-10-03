#pragma once

#include <cstddef>
#include <string>
#include <tuple>
#include <vector>

namespace map_controller
{
class SteeringLookup
{
public:
  explicit SteeringLookup(const std::string & table_name);

  double lookup(double lateral_accel, double longitudinal_speed) const;

private:
  struct NeighborPair
  {
    double primary_value{};
    std::size_t primary_index{};
    double secondary_value{};
    std::size_t secondary_index{};
  };

  void loadTable(const std::string & table_name);
  std::size_t findNearestIndex(const std::vector<double> & axis, double value) const;
  NeighborPair findClosestNeighbors(const std::vector<double> & values, double target) const;

  std::string table_name_;
  std::vector<double> speed_axis_;
  std::vector<double> steering_axis_;
  std::vector<std::vector<double>> accel_table_;
};
}  // namespace map_controller
