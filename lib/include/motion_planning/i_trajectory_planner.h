///
/// @file i_trajectory_planner.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_
#define MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_

#include <vector>

#include "motion_planning/domain_model/trajectory.h"
#include "motion_planning/domain_model/vehicle_dynamics.h"
#include "motion_planning/maneuver.h"

namespace motion_planning
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
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_
