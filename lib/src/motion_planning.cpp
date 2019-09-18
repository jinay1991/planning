///
/// @file
///

#include <logging/logging.h>
#include <motion_planning/maneuver_generator.h>
#include <motion_planning/motion_planning.h>
#include <motion_planning/trajectory_evaluator.h>
#include <motion_planning/trajectory_planner.h>
#include <motion_planning/trajectory_prioritizer.h>
#include <motion_planning/trajectory_selector.h>

namespace motion_planning
{
MotionPlanning::MotionPlanning(std::shared_ptr<IDataSource>& data_source)
    : maneuver_generator_{std::make_unique<ManeuverGenerator>()},
      trajectory_planner_{std::make_unique<TrajectoryPlanner>(data_source)},
      trajectory_evaluator_{std::make_unique<TrajectoryEvaluator>(data_source)},
      trajectory_prioritizer_{std::make_unique<TrajectoryPrioritizer>()},
      trajectory_selector_{std::make_unique<TrajectorySelector>()}
{
}

void MotionPlanning::GenerateTrajectories()
{
    const auto maneuvers = maneuver_generator_->Generate(units::velocity::meters_per_second_t{20});

    planned_trajectories_ = trajectory_planner_->GetPlannedTrajectories(maneuvers);

    rated_trajectories_ = trajectory_evaluator_->GetRatedTrajectories(planned_trajectories_);

    prioritized_trajectories_ = trajectory_prioritizer_->GetPrioritizedTrajectories(rated_trajectories_);

    selected_trajectory_ = trajectory_selector_->GetSelectedTrajectory(prioritized_trajectories_);
}

Trajectory MotionPlanning::GetSelectedTrajectory() const { return selected_trajectory_; }

}  // namespace motion_planning
