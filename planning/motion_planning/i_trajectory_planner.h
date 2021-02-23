///
/// @file
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_
#define PLANNING_MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_

#include "planning/datatypes/trajectory.h"
#include "planning/datatypes/vehicle_dynamics.h"
#include "planning/motion_planning/maneuver.h"

#include <vector>

namespace planning
{
/// @brief Interface for Trajectory Planner
class ITrajectoryPlanner
{
  public:
    /// @brief Destructor
    virtual ~ITrajectoryPlanner() = default;

    /// @brief Get Planned Trajectories for each maneuvers provided.
    virtual Trajectories GetPlannedTrajectories(const std::vector<Maneuver>& maneuvers) const = 0;
};
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_
