///
/// @file
///

#include <logging/logging.h>
#include <motion_planning/maneuver_generator.h>
#include <motion_planning/motion_planning.h>
#include <motion_planning/trajectory_planner.h>
#include <motion_planning/trajectory_prioritizer.h>
#include <motion_planning/trajectory_selector.h>

namespace motion_planning
{
MotionPlanning::MotionPlanning()
{
    maneuver_generator_ = std::make_unique<ManeuverGenerator>();
    trajectory_planner_ = std::make_unique<TrajectoryPlanner>();
    trajectory_prioritizer_ = std::make_unique<TrajectoryPrioritizer>();
    trajectory_selector_ = std::make_unique<TrajectorySelector>();
}

void MotionPlanning::GenerateTrajectories()
{
    const auto maneuvers = maneuver_generator_->Generate(units::velocity::meters_per_second_t{20});

    planned_trajectories_ = trajectory_planner_->GetPlannedTrajectories(maneuvers);

    prioritized_trajectories_ = trajectory_prioritizer_->GetPrioritizedTrajectories(planned_trajectories_);

    selected_trajectory_ = trajectory_selector_->GetSelectedTrajectory(prioritized_trajectories_);
}

void MotionPlanning::SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics)
{
    trajectory_planner_->SetVehicleDynamics(vehicle_dynamics);
}

void MotionPlanning::SetMapCoordinates(const std::vector<MapCoordinates>& map_coordinates)
{
    trajectory_planner_->SetMapCoordinates(map_coordinates);
}
void MotionPlanning::SetPreviousPath(const std::vector<GlobalCoordinates>& previous_path_global)
{
    trajectory_planner_->SetPreviousPath(previous_path_global);
}
void MotionPlanning::SetPreviousPath(const std::vector<FrenetCoordinates>& previous_path_frenet)
{
    trajectory_planner_->SetPreviousPath(previous_path_frenet);
}

Trajectory MotionPlanning::GetSelectedTrajectory() const { return selected_trajectory_; }

}  // namespace motion_planning
