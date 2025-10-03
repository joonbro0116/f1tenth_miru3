#include "map_controller/map_controller.hpp"

namespace map_controller
{
MapController::MapController(const MapControllerParams & params)
: params_(params)
{
}

ControllerOutput MapController::computeCommand(const VehicleState & state,
                                               const std::vector<Waypoint> & local_path)
{
  (void)state;
  (void)local_path;
  ControllerOutput out{};
  return out;
}
}  // namespace map_controller
