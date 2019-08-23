///
/// @file
///
#include <motion_planning/maneuver.h>

namespace motion_planning
{
Maneuver::Maneuver() : id_{LaneId::kEgo}, velocity_{units::velocity::meters_per_second_t{0.0}} {}

Maneuver::Maneuver(const LaneId& id, const units::velocity::meters_per_second_t& velocity)
    : id_{id}, velocity_{velocity}
{
}

}  // namespace motion_planning
