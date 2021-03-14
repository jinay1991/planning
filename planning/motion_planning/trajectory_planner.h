///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_TRAJECTORY_PLANNER_H
#define PLANNING_MOTION_PLANNING_TRAJECTORY_PLANNER_H

#include "planning/motion_planning/data_source.h"
#include "planning/motion_planning/i_trajectory_planner.h"

#include <spline.h>
#include <units.h>

#include <memory>

namespace planning
{
using Trajectories = std::vector<Trajectory>;

/// @brief Trajectory Planner
class TrajectoryPlanner : public ITrajectoryPlanner
{
  public:
    /// @brief Constructor. Initializes with provided DataSource
    explicit TrajectoryPlanner(const IDataSource& data_source);

    /// @brief Get Planned Trajectories for each maneuvers provided.
    Trajectories GetPlannedTrajectories(const std::vector<Maneuver>& maneuvers) const override;

  private:
    /// @brief Calculates initial waypoints for trajectory based on previous path/waypoints
    Trajectory GetInitialTrajectory() const;

    /// @brief Calculate Trajectory for given lane_id, target velocity
    Trajectory GetCalculatedTrajectory(const LaneId& lane_id) const;

    /// @brief Produces trajectories and optimizes for each maneuver
    Trajectories GetTrajectories(const std::vector<Maneuver>& maneuvers) const;

    /// @brief Converts Frenet Coordinates to Global Coordinates (using map)
    GlobalCoordinates GetGlobalCoordinates(const FrenetCoordinates& frenet_coords) const;

    /// @brief Converts Local Lane Id to Global Lane Id (using ego's global lane)
    GlobalLaneId GetGlobalLaneId(const LaneId& lane_id) const;

    /// @brief DataSource (contains information on VehicleDynamics, SensorFusion, etc.)
    const IDataSource& data_source_;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TRAJECTORY_PLANNER_H
