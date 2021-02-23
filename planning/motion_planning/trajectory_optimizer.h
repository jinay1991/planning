///
/// @file
/// @copyright Copyright (c) 2020. All Rights Reserved.
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
    explicit TrajectoryOptimizer(std::shared_ptr<IDataSource>& data_source);

    /// @brief Destructor.
    ~TrajectoryOptimizer() override;

    /// @brief Provide Optimized Trajectories set using Spline Equations
    virtual Trajectories GetOptimizedTrajectories(const Trajectories& planned_trajectories) const override;

  private:
    /// @brief Smoothen/Optimize Trajectory with Spline for target_velocity
    virtual Trajectory GetOptimizedTrajectory(const Trajectory& planned_trajectory) const;

    /// @brief DataSource (contains information on VehicleDynamics, SensorFusion, Map Points etc.)
    std::shared_ptr<IDataSource> data_source_;
};

}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TRAJECTORY_OPTIMIZER_H_
