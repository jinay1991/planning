///
/// @file
///
#ifndef MOTION_PLANNING_MOTION_PLANNING_H_
#define MOTION_PLANNING_MOTION_PLANNING_H_

#include <memory>

#include "motion_planning/domain_model/lane.h"
#include "motion_planning/domain_model/sensor_fusion.h"
#include "motion_planning/domain_model/trajectory.h"
#include "motion_planning/domain_model/vehicle_dynamics.h"
#include "motion_planning/i_data_source.h"
#include "motion_planning/i_maneuver.h"
#include "motion_planning/i_maneuver_generator.h"
#include "motion_planning/i_trajectory_evaluator.h"
#include "motion_planning/i_trajectory_optimizer.h"
#include "motion_planning/i_trajectory_planner.h"
#include "motion_planning/i_trajectory_prioritizer.h"
#include "motion_planning/i_trajectory_selector.h"
#include "motion_planning/velocity_planner/velocity_planner.h"

namespace motion_planning
{
class MotionPlanning
{
  public:
    explicit MotionPlanning(std::shared_ptr<IDataSource>& data_source);

    virtual void GenerateTrajectories();

    virtual Trajectory GetSelectedTrajectory() const;

  private:
    std::shared_ptr<IDataSource> data_source_;

    std::unique_ptr<VelocityPlanner> velocity_planner_;
    std::unique_ptr<IManeuverGenerator> maneuver_generator_;
    std::unique_ptr<ITrajectoryPlanner> trajectory_planner_;
    std::unique_ptr<ITrajectoryOptimizer> trajectory_optimizer_;
    std::unique_ptr<ITrajectoryEvaluator> trajectory_evaluator_;
    std::unique_ptr<ITrajectoryPrioritizer> trajectory_prioritizer_;
    std::unique_ptr<ITrajectorySelector> trajectory_selector_;

    Trajectory selected_trajectory_{};
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_MOTION_PLANNING_H_
