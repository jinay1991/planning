///
/// @file
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_TRAJECTORY_OPTIMIZER_H_
#define PLANNING_MOTION_PLANNING_TRAJECTORY_OPTIMIZER_H_

#include "planning/motion_planning/i_data_source.h"
#include "planning/motion_planning/i_trajectory_optimizer.h"

#include <spline.h>

#include <memory>

namespace planning
{
/// @brief Trajectory Optimizer
class TrajectoryOptimizer : public ITrajectoryOptimizer
{
  public:
    /// @brief Constructor. Initializes with provided DataSource
    explicit TrajectoryOptimizer(const IDataSource& data_source);

    /// @brief Provide Optimized Trajectories set using Spline Equations
    Trajectories GetOptimizedTrajectories(const Trajectories& planned_trajectories) const override;

  private:
    /// @brief Smoothen/Optimize Trajectory with Spline for target_velocity
    Trajectory GetOptimizedTrajectory(const Trajectory& planned_trajectory) const;

    /// @brief DataSource (contains information on VehicleDynamics, SensorFusion, Map Points etc.)
    const IDataSource& data_source_;
};

}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TRAJECTORY_OPTIMIZER_H_
