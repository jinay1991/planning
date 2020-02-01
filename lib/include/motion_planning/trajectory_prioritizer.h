///
/// @file trajectory_prioritizer.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_TRAJECTORY_PRIORITIZER_H_
#define MOTION_PLANNING_TRAJECTORY_PRIORITIZER_H_

#include "motion_planning/i_trajectory_prioritizer.h"

namespace motion_planning
{
/// @brief Trajectory Prioritizer
class TrajectoryPrioritizer : public ITrajectoryPrioritizer
{
  public:
    /// @brief Get Prioritized Trajectories for provided trajectories.
    virtual PrioritizedTrajectories GetPrioritizedTrajectories(const Trajectories& trajectories) const override;
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_TRAJECTORY_PRIORITIZER_H_
