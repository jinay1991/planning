///
/// @file
///
#ifndef MOTION_PLANNING_TRAJECTORY_SELECTOR_H_
#define MOTION_PLANNING_TRAJECTORY_SELECTOR_H_

#include <motion_planning/i_trajectory_selector.h>

namespace motion_planning
{
class TrajectorySelector : public ITrajectorySelector
{
  public:
    Trajectory GetSelectedTrajectory(const PrioritizedTrajectories prioritized_trajectories) const override;
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_TRAJECTORY_SELECTOR_H_
