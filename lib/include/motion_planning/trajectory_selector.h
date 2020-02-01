///
/// @file trajectory_selector.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_TRAJECTORY_SELECTOR_H_
#define MOTION_PLANNING_TRAJECTORY_SELECTOR_H_

#include "motion_planning/i_trajectory_selector.h"

namespace motion_planning
{
/// @brief Trajectory Selector
class TrajectorySelector : public ITrajectorySelector
{
  public:
    /// @brief Get Selected Trajectory from provided prioritized trajectories. (Selects top prioritized trajectory)
    virtual Trajectory GetSelectedTrajectory(const PrioritizedTrajectories& prioritized_trajectories) const override;
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_TRAJECTORY_SELECTOR_H_
