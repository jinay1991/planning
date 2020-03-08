///
/// @file i_trajectory_optimizer.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_I_TRAJECTORY_OPTIMIZER_H_
#define PLANNING_MOTION_PLANNING_I_TRAJECTORY_OPTIMIZER_H_

#include "planning/datatypes/trajectory.h"

#include <vector>

namespace planning
{
/// @brief Interface for Trajectory Optimizer
class ITrajectoryOptimizer
{
  public:
    /// @brief Destructor
    virtual ~ITrajectoryOptimizer() = default;

    /// @brief Get Optimized Trajectories for all the trajectories provided.
    virtual Trajectories GetOptimizedTrajectories(const Trajectories& trajectories) const = 0;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_I_TRAJECTORY_OPTIMIZER_H_