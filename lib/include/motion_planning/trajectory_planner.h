///
/// @file
///
#ifndef MOTION_PLANNING_TRAJECTORY_PLANNER_H_
#define MOTION_PLANNING_TRAJECTORY_PLANNER_H_

#include <spline.h>
#include <memory>

#include "motion_planning/i_data_source.h"
#include "motion_planning/i_trajectory_planner.h"

namespace motion_planning
{
using Trajectories = std::vector<Trajectory>;

class TrajectoryPlanner : public ITrajectoryPlanner
{
  public:
    explicit TrajectoryPlanner(std::shared_ptr<IDataSource>& data_source);

    /// @brief Provide Planned Trajectories
    virtual PlannedTrajectories GetPlannedTrajectories(const std::vector<Maneuver>& maneuvers) const override;

  private:
    /// @brief Calculates intial waypoints for trajectory based on previous path/waypoints
    virtual Trajectory GetInitialTrajectory() const;

    /// @brief Smoothen/Optimize Trajectory with Spline for target_velocity
    virtual Trajectory GetOptimizedTrajectory(const Trajectory& calculated_trajectory,
                                              const units::velocity::meters_per_second_t& target_velocity) const;

    /// @brief Calculate Trajectory for given lane_id, target velocity
    virtual Trajectory GetCalculatedTrajectory(const LaneId& lane_id) const;

    /// @brief Produces trajectories and optimizes for each maneuver
    virtual Trajectories GetTrajectories(const std::vector<Maneuver>& maneuvers) const;

    /// @brief Converts Frenet Coordinates to Global Coordinates (using map)
    virtual GlobalCoordinates GetGlobalCoordinates(const FrenetCoordinates& frenet_coords) const;

    /// @brief Converts Local Lane Id to Global Lane Id (using ego's global lane)
    virtual GlobalLaneId GetGlobalLaneId(const LaneId& lane_id) const;

    std::shared_ptr<IDataSource> data_source_;
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_TRAJECTORY_PLANNER_H_
