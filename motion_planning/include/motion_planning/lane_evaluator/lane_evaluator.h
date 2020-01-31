///
/// @file
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
    /// @brief Constructor (Initializes data_source)
    /// @param [in] data_source - DataSource for Map Inputs and Ego Positions
    explicit LaneEvaluator(std::shared_ptr<IDataSource>& data_source);

    /// @brief Evaluates Lane to be drivable (collision free)
    /// @param [in] lane_id - LaneId (local) to be evaluated for Collision Free
    /// @return TRUE if lane_id is collision free, otherwise FALSE
    virtual bool IsDrivableLane(const LaneId& lane_id) const;

    /// @brief Evaluates Lane to be Valid Lane
    /// @param [in] lane_id - LaneId (local) to be evaluated
    /// @return TRUE if land_id is Valid, otherwise FALSE
    virtual bool IsValidLane(const LaneId& lane_id) const;

  private:
    /// @brief Converts Global Lane Id to Local Lane Id based on Ego Position
    /// @param [in] global_lane_id - LaneId (global) to be converted
    /// @return Valid LaneId (local) if Lane Exists, otherwise Invalid LaneId (local)
    virtual LaneId GetLocalLaneId(const GlobalLaneId& global_lane_id) const;

    /// @brief Evaluates Euclidean Distance to Object Position from Ego Position
    /// @param [in] ego_position - Ego Position in Frenet Coordinates
    /// @param [in] obj_position - Object Position in Frenet Coordinates
    /// @return TRUE if Object is near to Ego (< 30m), otherwise FALSE
    virtual bool IsObjectNear(const FrenetCoordinates& ego_position, const FrenetCoordinates& obj_position) const;

    /// @brief Shared DataSource Object
    std::shared_ptr<IDataSource> data_source_;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_LANE_EVALUATOR_H_
