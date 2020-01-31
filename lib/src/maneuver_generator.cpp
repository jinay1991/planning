///
/// @file maneuver_generator.cpp
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include "motion_planning/maneuver_generator.h"

namespace motion_planning
{
std::vector<Maneuver> ManeuverGenerator::Generate(const units::velocity::meters_per_second_t& target_velocity) const
{
    return std::vector<Maneuver>{Maneuver{LaneId::kLeft, target_velocity}, Maneuver{LaneId::kEgo, target_velocity},
                                 Maneuver{LaneId::kRight, target_velocity}};
}
}  // namespace motion_planning
