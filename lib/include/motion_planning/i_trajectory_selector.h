///
/// @file
///
#ifndef MOTION_PLANNING_I_TRAJECTORY_SELECTOR_H_
#define MOTION_PLANNING_I_TRAJECTORY_SELECTOR_H_

#include <motion_planning/domain_model/trajectory.h>
#include <motion_planning/i_trajectory_prioritizer.h>

namespace motion_planning
{
class ITrajectorySelector
{
  public:
    virtual Trajectory GetSelectedTrajectory(const PrioritizedTrajectories& prioritized_trajectories) const = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_TRAJECTORY_SELECTOR_H_
