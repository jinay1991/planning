///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/maneuver.h"

namespace planning
{
Maneuver::Maneuver() : lane_id_{LaneId::kEgo}, velocity_{units::velocity::meters_per_second_t{0.0}} {}

Maneuver::Maneuver(const LaneId lane_id, const units::velocity::meters_per_second_t velocity)
    : lane_id_{lane_id}, velocity_{velocity}
{
}

LaneId Maneuver::GetLaneId() const
{
    return lane_id_;
}

units::velocity::meters_per_second_t Maneuver::GetVelocity() const
{
    return velocity_;
}

}  // namespace planning
