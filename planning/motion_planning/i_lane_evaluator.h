///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_MOTION_PLANNING_I_LANE_EVALUATOR_H
#define PLANNING_MOTION_PLANNING_I_LANE_EVALUATOR_H

#include "planning/datatypes/lane.h"

namespace planning
{
/// @brief Evaluates given Lane to be collision free
class ILaneEvaluator
{
  public:
    /// @brief Destructor
    virtual ~ILaneEvaluator() = default;

    /// @brief Evaluates Lane to be drivable (collision free)
    virtual bool IsDrivableLane(const LaneId lane_id) const = 0;

    /// @brief Evaluates Lane to be Valid Lane
    virtual bool IsValidLane(const LaneId lane_id) const = 0;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_I_LANE_EVALUATOR_H
