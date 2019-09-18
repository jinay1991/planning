///
/// @file
///
#ifndef MOTION_PLANNING_TRAJECTORY_PLANNER_H_
#define MOTION_PLANNING_TRAJECTORY_PLANNER_H_

#include <logging/logging.h>
#include <motion_planning/i_data_source.h>
#include <motion_planning/i_trajectory_planner.h>
#include <spline.h>
#include <memory>

namespace motion_planning
{
using Trajectories = std::vector<Trajectory>;

class TrajectoryPlanner : public ITrajectoryPlanner
{
  public:
    explicit TrajectoryPlanner(std::shared_ptr<IDataSource>& data_source);

    /// @brief Provide Planned Trajectories
    PlannedTrajectories GetPlannedTrajectories(const std::vector<Maneuver>& maneuvers) const override;

  private:
    /// @brief Calculates intial waypoints for trajectory based on previous path/waypoints
    Trajectory GetInitialTrajectory() const;

    /// @brief Smoothen/Optimize Trajectory with Spline for target_velocity
    Trajectory GetOptimizedTrajectory(const Trajectory& calculated_trajectory,
                                      const units::velocity::meters_per_second_t& target_velocity) const;

    /// @brief Calculate Trajectory for given lane_id, target velocity
    Trajectory GetCalculatedTrajectory(const Maneuver::LaneId& lane_id) const;

    /// @brief Produces trajectories and optimizes for each maneuver
    Trajectories GetTrajectories(const std::vector<Maneuver>& maneuvers) const;

    /// @brief Converts Frenet Coordinates to Global Coordinates (using map)
    GlobalCoordinates GetGlobalCoordinates(const FrenetCoordinates& frenet_coords) const;

    /// @brief Value of PI = 22/7 = 3.14.... (use of M_PI)
    static constexpr inline double PI() { return M_PI; }

    std::shared_ptr<IDataSource> data_source_;
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_TRAJECTORY_PLANNER_H_
