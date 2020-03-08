///
/// @file lane_evaluator.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_LANE_EVALUATOR_H_
#define PLANNING_MOTION_PLANNING_LANE_EVALUATOR_H_

#include "planning/datatypes/lane.h"
#include "planning/datatypes/sensor_fusion.h"
#include "planning/datatypes/trajectory.h"
#include "planning/motion_planning/i_data_source.h"

#include <memory>

namespace planning
{
/// @brief Evaluates given Lane to be collision free
class LaneEvaluator
{
  public:
    /// @brief Constructor. Initializes based on provided DataSource
    explicit LaneEvaluator(std::shared_ptr<IDataSource>& data_source);

    /// @brief Evaluates Lane to be drivable (collision free)
    virtual bool IsDrivableLane(const LaneId& lane_id) const;

    /// @brief Evaluates Lane to be Valid Lane
    virtual bool IsValidLane(const LaneId& lane_id) const;

  private:
    /// @brief Converts Global Lane Id to Local Lane Id based on Ego Position
    virtual LaneId GetLocalLaneId(const GlobalLaneId& global_lane_id) const;

    /// @brief Evaluates Euclidean Distance to Object Position from Ego Position
    virtual bool IsObjectNear(const FrenetCoordinates& ego_position, const FrenetCoordinates& obj_position) const;

    /// @brief DataSource (contains information on VehicleDynamics, SensorFusion, etc.)
    std::shared_ptr<IDataSource> data_source_;
};
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_LANE_EVALUATOR_H_
