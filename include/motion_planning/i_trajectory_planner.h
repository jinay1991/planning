///
/// @file
///

#ifndef MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_
#define MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_
namespace motion_planning
{
class ITrajectoryPlanner
{
  public:
    virtual ~ITrajectoryPlanner() = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_TRAJECTORY_PLANNER_H_