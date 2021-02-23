///
/// @file
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_I_TRAJECTORY_EVALUATOR_H_
#define PLANNING_MOTION_PLANNING_I_TRAJECTORY_EVALUATOR_H_

#include "planning/datatypes/sensor_fusion.h"
#include "planning/datatypes/trajectory.h"
#include "planning/motion_planning/i_trajectory_planner.h"

#include <vector>

namespace planning
{
/// @brief Interface for Trajectory Evaluator
class ITrajectoryEvaluator
{
  public:
    /// @brief Destructor
    virtual ~ITrajectoryEvaluator() = default;

    /// @brief Get Rated Trajectories for all the optimized trajectories.
    /// @note Invalid Trajectories will not be rated and are removed.
    virtual Trajectories GetRatedTrajectories(const Trajectories& optimized_trajectories) const = 0;
};
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_I_TRAJECTORY_EVALUATOR_H_
