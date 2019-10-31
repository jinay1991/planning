///
/// @file
///
#ifndef MOTION_PLANNING_TRAJECTORY_EVALUATOR_H_
#define MOTION_PLANNING_TRAJECTORY_EVALUATOR_H_

#include <motion_planning/domain_model/sensor_fusion.h>
#include <motion_planning/i_data_source.h>
#include <motion_planning/i_trajectory_evaluator.h>
#include <motion_planning/lane_evaluator/lane_evaluator.h>
#include <algorithm>
#include <memory>

namespace motion_planning
{
class TrajectoryEvaluator : public ITrajectoryEvaluator
{
  public:
    explicit TrajectoryEvaluator(std::shared_ptr<IDataSource>& data_source);

    RatedTrajectories GetRatedTrajectories(const PlannedTrajectories& planned_trajectories) const override;

  private:
    std::unique_ptr<LaneEvaluator> lane_evaluator_;
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_TRAJECTORY_EVALUATOR_H_
