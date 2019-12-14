///
/// @file
///
#ifndef MOTION_PLANNING_I_TRAJECTORY_PRIORITIZER_H_
#define MOTION_PLANNING_I_TRAJECTORY_PRIORITIZER_H_

#include <queue>

#include "motion_planning/domain_model/trajectory.h"
#include "motion_planning/i_trajectory_planner.h"

namespace motion_planning
{
using PrioritizedTrajectories = std::priority_queue<Trajectory, std::vector<Trajectory>, std::greater<Trajectory>>;

class ITrajectoryPrioritizer
{
  public:
    virtual PrioritizedTrajectories GetPrioritizedTrajectories(const Trajectories& trajectories) const = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_TRAJECTORY_PRIORITIZER_H_
