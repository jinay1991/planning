///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_PLANNING_MOTION_PLANNING_H
#define PLANNING_MOTION_PLANNING_PLANNING_MOTION_PLANNING_H

#include "planning/datatypes/lane.h"
#include "planning/datatypes/sensor_fusion.h"
#include "planning/datatypes/trajectory.h"
#include "planning/datatypes/vehicle_dynamics.h"
#include "planning/motion_planning/data_source.h"
#include "planning/motion_planning/i_maneuver.h"
#include "planning/motion_planning/i_maneuver_generator.h"
#include "planning/motion_planning/i_trajectory_evaluator.h"
#include "planning/motion_planning/i_trajectory_optimizer.h"
#include "planning/motion_planning/i_trajectory_planner.h"
#include "planning/motion_planning/i_trajectory_prioritizer.h"
#include "planning/motion_planning/i_trajectory_selector.h"
#include "planning/motion_planning/i_velocity_planner.h"

#include <memory>

namespace planning
{
/// @brief Motion Planning Wrapper Class
class MotionPlanning
{
  public:
    /// @brief Constructor. Initialize Motion Planner with DataSource instance
    explicit MotionPlanning(const DataSource& data_source);

    /// @brief Generate Trajectories based on the provided DataSource (i.e. Environment)
    void GenerateTrajectories();

    /// @brief Get Selected Trajectory from Trajectory Selector
    Trajectory GetSelectedTrajectory() const;

  private:
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
    Trajectory selected_trajectory_;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_PLANNING_MOTION_PLANNING_H
