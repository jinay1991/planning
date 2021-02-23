///
/// @file
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_I_TRAJECTORY_SELECTOR_H_
#define PLANNING_MOTION_PLANNING_I_TRAJECTORY_SELECTOR_H_

#include "planning/datatypes/trajectory.h"
#include "planning/motion_planning/i_trajectory_prioritizer.h"

namespace planning
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
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_I_TRAJECTORY_SELECTOR_H_
