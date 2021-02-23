///
/// @file
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_I_TRAJECTORY_PRIORITIZER_H_
#define PLANNING_MOTION_PLANNING_I_TRAJECTORY_PRIORITIZER_H_

#include "planning/datatypes/trajectory.h"
#include "planning/motion_planning/i_trajectory_planner.h"

#include <queue>

namespace planning
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
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_I_TRAJECTORY_PRIORITIZER_H_
