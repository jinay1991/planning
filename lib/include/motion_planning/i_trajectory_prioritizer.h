///
/// @file i_trajectory_prioritizer.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_I_TRAJECTORY_PRIORITIZER_H_
#define MOTION_PLANNING_I_TRAJECTORY_PRIORITIZER_H_

#include <queue>

#include "motion_planning/domain_model/trajectory.h"
#include "motion_planning/i_trajectory_planner.h"

namespace motion_planning
{
/// @brief typename for prioritized queue. 
using PrioritizedTrajectories = std::priority_queue<Trajectory, std::vector<Trajectory>, std::greater<Trajectory>>;

/// @brief Interface for Trajectory Prioritizer
class ITrajectoryPrioritizer
{
  public:
    /// @brief Destructor
    virtual ~ITrajectoryPrioritizer() = default;

    /// @brief Get Prioritized Trajectories for rated trajectories provided.
    virtual PrioritizedTrajectories GetPrioritizedTrajectories(const Trajectories& trajectories) const = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_TRAJECTORY_PRIORITIZER_H_