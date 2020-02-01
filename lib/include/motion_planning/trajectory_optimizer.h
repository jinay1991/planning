///
/// @file trajectory_optimizer.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_TRAJECTORY_OPTIMIZER_H_
#define MOTION_PLANNING_TRAJECTORY_OPTIMIZER_H_

#include <spline.h>
#include <memory>

#include "motion_planning/i_data_source.h"
#include "motion_planning/i_trajectory_optimizer.h"

namespace motion_planning
{
/// @brief Trajectory Optimizer
class TrajectoryOptimizer : public ITrajectoryOptimizer
{
  public:
    /// @brief Constructor. Initializes with provided DataSource
    explicit TrajectoryOptimizer(std::shared_ptr<IDataSource>& data_source);

    /// @brief Provide Optimized Trajectories set using Spline Equations
    virtual Trajectories GetOptimizedTrajectories(const Trajectories& planned_trajectories) const override;

  private:
    /// @brief Smoothen/Optimize Trajectory with Spline for target_velocity
    virtual Trajectory GetOptimizedTrajectory(const Trajectory& planned_trajectory) const;

    /// @brief DataSource (contains information on VehicleDynamics, SensorFusion, Map Points etc.)
    std::shared_ptr<IDataSource> data_source_;
};

}  // namespace motion_planning

#endif  /// MOTION_PLANNING_TRAJECTORY_OPTIMIZER_H_