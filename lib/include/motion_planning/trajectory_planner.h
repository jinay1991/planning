///
/// @file
///
#ifndef MOTION_PLANNING_TRAJECTORY_PLANNER_H_
#define MOTION_PLANNING_TRAJECTORY_PLANNER_H_

#include <motion_planning/i_trajectory_planner.h>
#include <spline.h>

namespace motion_planning
{
using Trajectories = std::vector<Trajectory>;

class TrajectoryPlanner : public ITrajectoryPlanner
{
  public:
    /// @brief Provide Planned Trajectories
    PlannedTrajectories GetPlannedTrajectories(const std::vector<Maneuver> maneuvers) const override;

    void SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics) override;
    void SetMapCoordinates(const std::vector<MapCoordinates> map_coordinates) override;
    void SetPreviousPath(const std::vector<GlobalCoordinates> previous_path_global);
    void SetPreviousPath(const std::vector<FrenetCoordinates> previous_path_frenet);

  private:
    /// @brief Calculates intial waypoints for trajectory based on previous path/waypoints
    Trajectory GetInitialTrajectory() const;

    /// @brief Smoothen/Optimize Trajectory with Spline for target_velocity
    Trajectory GetOptimizedTrajectory(const Trajectory& trajectory,
                                      const units::velocity::meters_per_second_t& target_velocity) const;

    /// @brief Calculate Trajectory for given lane_id, target velocity
    Trajectory GetCalculatedTrajectory(const Maneuver::LaneId& lane_id) const;

    /// @brief Produces trajectories and optimizes for each maneuver
    Trajectories GetTrajectories(const std::vector<Maneuver> maneuvers) const;

    /// @brief Converts Frenet Coordinates to Global Coordinates (using map)
    GlobalCoordinates GetGlobalCoordinates(const FrenetCoordinates& frenet_coords) const;

    /// @brief Compute Lane Cost for given trajectory
    double GetLaneCost(const Maneuver::LaneId& lane_id) const;

    /// @brief Value of PI = 22/7 = 3.14.... (use of M_PI)
    constexpr inline double PI() const { return M_PI; }

    /// @brief Vehicle's current dynamics (i.e. velocity, position, etc.)
    VehicleDynamics vehicle_dynamics_;

    /// @brief Route information (based on map coordinates)
    std::vector<MapCoordinates> map_coordinates_;

    /// @brief Previous Route information (based on global coordinates)
    std::vector<GlobalCoordinates> previous_path_global_;

    /// @brief Previous Route information (based on frenet coordinates)
    std::vector<FrenetCoordinates> previous_path_frenet_;
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_TRAJECTORY_PLANNER_H_
