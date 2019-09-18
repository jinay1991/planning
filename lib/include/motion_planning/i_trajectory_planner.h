///
/// @file
///

#ifndef MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_
#define MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_

#include <motion_planning/domain_model/trajectory.h>
#include <motion_planning/domain_model/vehicle_dynamics.h>
#include <motion_planning/maneuver.h>
#include <vector>

namespace motion_planning
{
using PlannedTrajectories = std::vector<Trajectory>;

class ITrajectoryPlanner
{
  public:
    virtual PlannedTrajectories GetPlannedTrajectories(const std::vector<Maneuver>& maneuvers) const = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_
