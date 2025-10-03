#pragma once

#include <string>

namespace map_controller
{
class SteeringLookup
{
public:
  explicit SteeringLookup(const std::string & table_name);

  double lookup(double lateral_accel, double longitudinal_speed) const;

private:
  std::string table_name_;
};
}  // namespace map_controller
