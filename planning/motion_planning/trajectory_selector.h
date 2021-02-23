///
/// @file
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_TRAJECTORY_SELECTOR_H_
#define PLANNING_MOTION_PLANNING_TRAJECTORY_SELECTOR_H_

#include "planning/motion_planning/i_trajectory_selector.h"

namespace planning
{
/// @brief Trajectory Selector
class TrajectorySelector : public ITrajectorySelector
{
  public:
    /// @brief Destructor.
    ~TrajectorySelector() override;

    /// @brief Get Selected Trajectory from provided prioritized trajectories. (Selects top prioritized trajectory)
    virtual Trajectory GetSelectedTrajectory(const PrioritizedTrajectories& prioritized_trajectories) const override;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TRAJECTORY_SELECTOR_H_
