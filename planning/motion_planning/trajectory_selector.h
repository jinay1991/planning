///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_TRAJECTORY_SELECTOR_H
#define PLANNING_MOTION_PLANNING_TRAJECTORY_SELECTOR_H

#include "planning/motion_planning/i_trajectory_selector.h"

namespace planning
{
/// @brief Trajectory Selector
class TrajectorySelector : public ITrajectorySelector
{
  public:
    /// @brief Get Selected Trajectory from provided prioritized trajectories. (Selects top prioritized trajectory)
    Trajectory GetSelectedTrajectory(const PrioritizedTrajectories& prioritized_trajectories) const override;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TRAJECTORY_SELECTOR_H
