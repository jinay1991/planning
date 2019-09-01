///
/// @file
///
#ifndef MOTION_PLANNING_MOTION_PLANNING_H_
#define MOTION_PLANNING_MOTION_PLANNING_H_

#include <motion_planning/domain_model/lane.h>
#include <motion_planning/domain_model/trajectory.h>
#include <motion_planning/domain_model/vehicle_dynamics.h>
#include <motion_planning/i_data_source.h>
#include <motion_planning/i_maneuver.h>
#include <motion_planning/i_maneuver_generator.h>
#include <motion_planning/i_trajectory_planner.h>
#include <motion_planning/i_trajectory_prioritizer.h>
#include <motion_planning/i_trajectory_selector.h>
#include <motion_planning/maneuver.h>
#include <motion_planning/maneuver_generator.h>
#include <motion_planning/trajectory_planner.h>
#include <motion_planning/trajectory_prioritizer.h>
#include <motion_planning/trajectory_selector.h>

namespace motion_planning
{
class MotionPlanning
{
  public:
    MotionPlanning();

    ~MotionPlanning() = default;

    void GenerateTrajectories();

    void SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics);
    void SetMapCoordinates(const std::vector<MapCoordinates>& map_coordinates);
    void SetPreviousPath(const std::vector<GlobalCoordinates>& previous_path_global);
    void SetPreviousPath(const std::vector<FrenetCoordinates>& previous_path_frenet);

    Trajectory GetSelectedTrajectory() const;

  private:
    std::unique_ptr<IManeuverGenerator> maneuver_generator_;
    std::unique_ptr<ITrajectoryPlanner> trajectory_planner_;
    std::unique_ptr<ITrajectoryPrioritizer> trajectory_prioritizer_;
    std::unique_ptr<ITrajectorySelector> trajectory_selector_;

    PlannedTrajectories planned_trajectories_;
    PrioritizedTrajectories prioritized_trajectories_;
    Trajectory selected_trajectory_;
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_MOTION_PLANNING_H_
