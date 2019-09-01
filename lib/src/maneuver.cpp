///
/// @file
///
#include <motion_planning/maneuver.h>

namespace motion_planning
{
Maneuver::Maneuver() : id_{Maneuver::LaneId::kEgo}, velocity_{units::velocity::meters_per_second_t{0.0}} {}

Maneuver::Maneuver(const Maneuver::LaneId& id, const units::velocity::meters_per_second_t& velocity)
    : id_{id}, velocity_{velocity}
{
}

Maneuver::LaneId Maneuver::GetLaneId() const { return id_; }

units::velocity::meters_per_second_t Maneuver::GetVelocity() const { return velocity_; }

}  // namespace motion_planning
