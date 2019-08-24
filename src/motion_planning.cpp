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
    const auto maneuvers = maneuver_generator_->Generate(units::velocity::meters_per_second_t{14.0});
    // for (const auto maneuver: maneuvers)
    // {
    // }
    const auto rated_trajectories = trajectory_planner_->GetRatedTrajectories(maneuvers);
    const auto prioritized_trajectories = trajectory_prioritizer_->GetPrioritizedTrajectories(rated_trajectories);

    trajectory_selector_->SetSelectedTrajectory(prioritized_trajectories->GetHighestPriorityTrajectory());
}
}  // namespace motion_planning
