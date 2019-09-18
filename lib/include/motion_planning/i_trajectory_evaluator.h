///
/// @file
///

#ifndef MOTION_PLANNING_I_TRAJECTORY_EVALUATOR_H_
#define MOTION_PLANNING_I_TRAJECTORY_EVALUATOR_H_

#include <motion_planning/domain_model/sensor_fusion.h>
#include <motion_planning/domain_model/trajectory.h>
#include <motion_planning/i_trajectory_planner.h>
#include <vector>

namespace motion_planning
{
using RatedTrajectories = std::vector<Trajectory>;

class ITrajectoryEvaluator
{
  public:
    virtual RatedTrajectories GetRatedTrajectories(const PlannedTrajectories& planned_trajectories) const = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_TRAJECTORY_EVALUATOR_H_
