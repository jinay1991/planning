///
/// @file
///

#include <motion_planning/maneuver_generator.h>

namespace motion_planning
{
std::vector<IManeuver> ManeuverGenerator::Generate(const units::velocity::meters_per_second_t& target_velocity) const
{
    return std::vector<IManeuver>();
}
}  // namespace motion_planning