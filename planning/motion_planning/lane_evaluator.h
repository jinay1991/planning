///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_LANE_EVALUATOR_H
#define PLANNING_MOTION_PLANNING_LANE_EVALUATOR_H

#include "planning/datatypes/lane.h"
#include "planning/datatypes/sensor_fusion.h"
#include "planning/datatypes/trajectory.h"
#include "planning/motion_planning/data_source.h"

#include <memory>

namespace planning
{
/// @brief Evaluates given Lane to be collision free
class LaneEvaluator
{
  public:
    /// @brief Constructor. Initializes based on provided DataSource
    explicit LaneEvaluator(const IDataSource& data_source);

    /// @brief Evaluates Lane to be drivable (collision free)
    bool IsDrivableLane(const LaneId lane_id) const;

    /// @brief Evaluates Lane to be Valid Lane
    bool IsValidLane(const LaneId lane_id) const;

  private:
    /// @brief Converts Global Lane Id to Local Lane Id based on Ego Position
    LaneId GetLocalLaneId(const GlobalLaneId global_lane_id) const;

    /// @brief DataSource (contains information on VehicleDynamics, SensorFusion, etc.)
    const IDataSource& data_source_;
};
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_LANE_EVALUATOR_H
