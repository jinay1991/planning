///
/// @file i_trajectory_evaluator.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_I_TRAJECTORY_EVALUATOR_H_
#define MOTION_PLANNING_I_TRAJECTORY_EVALUATOR_H_

#include <vector>

#include "motion_planning/domain_model/sensor_fusion.h"
#include "motion_planning/domain_model/trajectory.h"
#include "motion_planning/i_trajectory_planner.h"

namespace motion_planning
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
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_TRAJECTORY_EVALUATOR_H_
