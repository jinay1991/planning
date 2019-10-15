///
/// @file
///

#ifndef MOTION_PLANNING_LANE_EVALUATOR_H_
#define MOTION_PLANNING_LANE_EVALUATOR_H_

#include <motion_planning/domain_model/lane.h>
#include <motion_planning/domain_model/sensor_fusion.h>
#include <motion_planning/domain_model/trajectory.h>
#include <motion_planning/i_data_source.h>
#include <memory>

namespace motion_planning
{
static const auto gkFarDistanceThreshold = units::length::meter_t{30.0};

class LaneEvaluator
{
  public:
    explicit LaneEvaluator(std::shared_ptr<IDataSource>& data_source);
    virtual ~LaneEvaluator() = default;

    bool IsDrivableLane(const LaneInformation::LaneId& lane_id) const;

    bool IsValidLane(const LaneInformation::LaneId& lane_id) const;

  private:
    bool IsLeftLane(const FrenetCoordinates& coords) const;
    bool IsCenterLane(const FrenetCoordinates& coords) const;
    bool IsRightLane(const FrenetCoordinates& coords) const;

    LaneInformation::GlobalLaneId GetGlobalLaneId(const FrenetCoordinates& coords) const;
    LaneInformation::LaneId GetLocalLaneId(const LaneInformation::GlobalLaneId& global_lane_id) const;
    bool IsObjectNear(const FrenetCoordinates& ego_position, const FrenetCoordinates& obj_position) const;

    std::shared_ptr<IDataSource> data_source_;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_LANE_EVALUATOR_H_
