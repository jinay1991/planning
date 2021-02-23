///
/// @file
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_TRAJECTORY_EVALUATOR_H_
#define PLANNING_MOTION_PLANNING_TRAJECTORY_EVALUATOR_H_

#include "planning/datatypes/sensor_fusion.h"
#include "planning/motion_planning/i_data_source.h"
#include "planning/motion_planning/i_trajectory_evaluator.h"
#include "planning/motion_planning/lane_evaluator.h"

#include <algorithm>
#include <memory>

namespace planning
{
/// @brief Trajectory Evaluator
class TrajectoryEvaluator : public ITrajectoryEvaluator
{
  public:
    /// @brief Constructor. Initializes with provided DataSource
    explicit TrajectoryEvaluator(const IDataSource& data_source);

    /// @brief Get Rated Trajectories for provided optimized trajectories.
    Trajectories GetRatedTrajectories(const Trajectories& optimized_trajectories) const override;

  private:
    /// @brief Lane Evaluator
    std::unique_ptr<LaneEvaluator> lane_evaluator_;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TRAJECTORY_EVALUATOR_H_
