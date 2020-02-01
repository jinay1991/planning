///
/// @file i_trajectory_selector.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_I_TRAJECTORY_SELECTOR_H_
#define MOTION_PLANNING_I_TRAJECTORY_SELECTOR_H_

#include "motion_planning/domain_model/trajectory.h"
#include "motion_planning/i_trajectory_prioritizer.h"

namespace motion_planning
{
/// @brief Interface for Trajectory Selector
class ITrajectorySelector
{
  public:
    /// @brief Destructor
    virtual ~ITrajectorySelector() = default;

    /// @brief Get Selected Trajectory from the prioritized trajectories provided.
    virtual Trajectory GetSelectedTrajectory(const PrioritizedTrajectories& prioritized_trajectories) const = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_TRAJECTORY_SELECTOR_H_
