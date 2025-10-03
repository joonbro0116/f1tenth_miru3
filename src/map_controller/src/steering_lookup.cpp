#include "map_controller/steering_lookup.hpp"

namespace map_controller
{
SteeringLookup::SteeringLookup(const std::string & table_name)
: table_name_(table_name)
{
}

double SteeringLookup::lookup(double lateral_accel, double longitudinal_speed) const
{
  (void)lateral_accel;
  (void)longitudinal_speed;
  return 0.0;
}
}  // namespace map_controller
