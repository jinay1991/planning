///
/// @file
///
#ifndef MOTION_PLANNING_I_TRAJECTORY_OPTIMIZER_H_
#define MOTION_PLANNING_I_TRAJECTORY_OPTIMIZER_H_

#include <vector>

#include "motion_planning/domain_model/trajectory.h"

namespace motion_planning
{
class ITrajectoryOptimizer
{
  public:
    virtual Trajectories GetOptimizedTrajectories(const Trajectories& trajectories) const = 0;
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_I_TRAJECTORY_OPTIMIZER_H_