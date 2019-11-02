///
/// @file
///

#ifndef MOTION_PLANNING_LANE_EVALUATOR_H_
#define MOTION_PLANNING_LANE_EVALUATOR_H_

#include <units.h>
#include <memory>

#include "motion_planning/domain_model/lane.h"
#include "motion_planning/domain_model/sensor_fusion.h"
#include "motion_planning/domain_model/trajectory.h"
#include "motion_planning/i_data_source.h"

namespace motion_planning
{
class LaneEvaluator
{
  public:
    explicit LaneEvaluator(std::shared_ptr<IDataSource>& data_source);
    virtual ~LaneEvaluator() = default;

    virtual bool IsDrivableLane(const LaneInformation::LaneId& lane_id) const;

    virtual bool IsValidLane(const LaneInformation::LaneId& lane_id) const;

  private:
    virtual LaneInformation::LaneId GetLocalLaneId(const LaneInformation::GlobalLaneId& global_lane_id) const;
    virtual bool IsObjectNear(const FrenetCoordinates& ego_position, const FrenetCoordinates& obj_position) const;

    std::shared_ptr<IDataSource> data_source_;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_LANE_EVALUATOR_H_
