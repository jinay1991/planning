///
/// @file
///

#include <motion_planning/maneuver_generator.h>

namespace motion_planning
{
std::vector<Maneuver> ManeuverGenerator::Generate(const units::velocity::meters_per_second_t& target_velocity) const
{
    return std::vector<Maneuver>{Maneuver{Maneuver::LaneId::kEgo, target_velocity},
                                 Maneuver{Maneuver::LaneId::kLeft, target_velocity},
                                 Maneuver{Maneuver::LaneId::kRight, target_velocity}};
}
}  // namespace motion_planning