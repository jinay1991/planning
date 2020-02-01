///
/// @file motion_planning.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
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
#include "motion_planning/velocity_planner/i_velocity_planner.h"

namespace motion_planning
{
/// @brief Motion Planning Wrapper Class
class MotionPlanning
{
  public:
    /// @brief Constructor. Initialize Motion Planner with DataSource instance
    explicit MotionPlanning(std::shared_ptr<IDataSource>& data_source);

    /// @brief Generate Trajectories based on the provided DataSource (i.e. Environment)
    virtual void GenerateTrajectories();

    /// @brief Get Selected Trajectory from Trajectory Selector
    virtual Trajectory GetSelectedTrajectory() const;

  private:
    /// @brief DataSource (contains information on Environment, VehicleDynamics, Map Points, SensorFusion etc.)
    std::shared_ptr<IDataSource> data_source_;

    /// @brief Velocity Planner
    std::unique_ptr<IVelocityPlanner> velocity_planner_;

    /// @brief Maneuver Generator
    std::unique_ptr<IManeuverGenerator> maneuver_generator_;

    /// @brief Trajectory Planner
    std::unique_ptr<ITrajectoryPlanner> trajectory_planner_;

    /// @brief Trajectory Optimizer
    std::unique_ptr<ITrajectoryOptimizer> trajectory_optimizer_;

    /// @brief Trajectory Evaluator
    std::unique_ptr<ITrajectoryEvaluator> trajectory_evaluator_;

    /// @brief Trajectory Prioritizer
    std::unique_ptr<ITrajectoryPrioritizer> trajectory_prioritizer_;

    /// @brief Trajectory Selector
    std::unique_ptr<ITrajectorySelector> trajectory_selector_;

    /// @brief Selected Trajectory
    Trajectory selected_trajectory_{};
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_MOTION_PLANNING_H_
