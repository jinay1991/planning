///
/// @file
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_TRAJECTORY_PRIORITIZER_H_
#define PLANNING_MOTION_PLANNING_TRAJECTORY_PRIORITIZER_H_

#include "planning/motion_planning/i_trajectory_prioritizer.h"

namespace planning
{
/// @brief Trajectory Prioritizer
class TrajectoryPrioritizer : public ITrajectoryPrioritizer
{
  public:
    /// @brief Get Prioritized Trajectories for provided trajectories.
    PrioritizedTrajectories GetPrioritizedTrajectories(const Trajectories& trajectories) const override;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TRAJECTORY_PRIORITIZER_H_
