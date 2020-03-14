///
/// @file trajectory_prioritizer.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
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
    /// @brief Destructor.
    ~TrajectoryPrioritizer() override;

    /// @brief Get Prioritized Trajectories for provided trajectories.
    virtual PrioritizedTrajectories GetPrioritizedTrajectories(const Trajectories& trajectories) const override;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TRAJECTORY_PRIORITIZER_H_
