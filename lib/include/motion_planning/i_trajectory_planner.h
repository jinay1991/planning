///
/// @file
///

#ifndef MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_
#define MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_

#include <motion_planning/maneuver.h>
#include <vector>

namespace motion_planning
{
struct RatedTrajectory
{
    units::velocity::meters_per_second_t velocity;
    Maneuver::LaneId lane_id;
    double cost;
};
class ITrajectoryPlanner
{
  public:
    virtual std::vector<RatedTrajectory> GetRatedTrajectories(const std::vector<Maneuver> maneuvers) const = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_
