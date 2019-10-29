///
/// @file
///
#include <motion_planning/motion_planning.h>

namespace motion_planning
{
MotionPlanning::MotionPlanning(std::shared_ptr<IDataSource>& data_source)
    : velocity_planner_{std::make_unique<VelocityPlanner>(data_source)},
      maneuver_generator_{std::make_unique<ManeuverGenerator>()},
      trajectory_planner_{std::make_unique<TrajectoryPlanner>(data_source)},
      trajectory_evaluator_{std::make_unique<TrajectoryEvaluator>(data_source)},
      trajectory_prioritizer_{std::make_unique<TrajectoryPrioritizer>()},
      trajectory_selector_{std::make_unique<TrajectorySelector>()}
{
}

void MotionPlanning::GenerateTrajectories()
{
    velocity_planner_->CalculateTargetVelocity();
    const auto target_velocity = velocity_planner_->GetTargetVelocity();

    const auto maneuvers = maneuver_generator_->Generate(target_velocity);

    planned_trajectories_ = trajectory_planner_->GetPlannedTrajectories(maneuvers);

    rated_trajectories_ = trajectory_evaluator_->GetRatedTrajectories(planned_trajectories_);

    prioritized_trajectories_ = trajectory_prioritizer_->GetPrioritizedTrajectories(rated_trajectories_);

    selected_trajectory_ = trajectory_selector_->GetSelectedTrajectory(prioritized_trajectories_);
}

Trajectory MotionPlanning::GetSelectedTrajectory() const { return selected_trajectory_; }

}  // namespace motion_planning
