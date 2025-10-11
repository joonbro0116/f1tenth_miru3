#pragma once

#include <optional>
#include <string>
#include <vector>

namespace map_controller
{
class SteeringLookup
{
public:
  explicit SteeringLookup(const std::string & lut_name);

  std::optional<double> lookup(double lateral_accel, double velocity_mps) const;

private:
  std::vector<double> velocities_;
  std::vector<double> steer_angles_;
  std::vector<std::vector<double>> table_;
};

}  // namespace map_controller
