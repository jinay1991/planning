///
/// @file
///
#ifndef MOTION_PLANNING_MANEUVER_H_
#define MOTION_PLANNING_MANEUVER_H_

#include <motion_planning/domain_model/lane.h>
#include <motion_planning/i_maneuver.h>
#include <units.h>

namespace motion_planning
{
class Maneuver : public IManeuver
{
  public:
    using LaneId = LaneInformation::LaneId;

    Maneuver();

    Maneuver(const LaneId& id, const units::velocity::meters_per_second_t& velocity);

    LaneId GetLaneId() const;

    units::velocity::meters_per_second_t GetVelocity() const;

    inline bool operator==(const Maneuver& rhs) const { return id_ == rhs.id_ && velocity_ == rhs.velocity_; };

  private:
    LaneId id_;

    units::velocity::meters_per_second_t velocity_;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_MANEUVER_H_
