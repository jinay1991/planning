///
/// @file
///
#ifndef MOTION_PLANNING_TRAJECTORY_PLANNER_H_
#define MOTION_PLANNING_TRAJECTORY_PLANNER_H_

#include <motion_planning/i_trajectory_planner.h>

namespace motion_planning
{
class TrajectoryPlanner : public ITrajectoryPlanner
{
  public:
    std::vector<RatedTrajectory> GetRatedTrajectories(const std::vector<Maneuver> maneuvers) const override{};
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_TRAJECTORY_PLANNER_H_
