///
/// @file
///
#ifndef MOTION_PLANNING_MANEUVER_H_
#define MOTION_PLANNING_MANEUVER_H_

#include "motion_planning/i_maneuver.h"

namespace motion_planning
{
class Maneuver : public IManeuver
{
  public:
    Maneuver();
    explicit Maneuver(const LaneId& lane_id, const units::velocity::meters_per_second_t& velocity);

    LaneId GetLaneId() const;

    units::velocity::meters_per_second_t GetVelocity() const;

    inline bool operator==(const Maneuver& rhs) const { return lane_id_ == rhs.lane_id_ && velocity_ == rhs.velocity_; }

  private:
    LaneId lane_id_;

    units::velocity::meters_per_second_t velocity_;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_MANEUVER_H_
