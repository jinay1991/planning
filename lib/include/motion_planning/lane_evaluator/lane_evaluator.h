///
/// @file lane_evaluator.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_LANE_EVALUATOR_H_
#define MOTION_PLANNING_LANE_EVALUATOR_H_

#include <units.h>
#include <chrono>
#include <memory>
#include <sstream>

#include "motion_planning/domain_model/lane.h"
#include "motion_planning/domain_model/sensor_fusion.h"
#include "motion_planning/domain_model/trajectory.h"
#include "motion_planning/i_data_source.h"

namespace motion_planning
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
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_LANE_EVALUATOR_H_
