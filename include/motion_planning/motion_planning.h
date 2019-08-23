///
/// @file
///
#ifndef MOTION_PLANNING_MOTION_PLANNING_H_
#define MOTION_PLANNING_MOTION_PLANNING_H_

#include <motion_planning/i_data_source.h>
#include <motion_planning/i_maneuver.h>
#include <motion_planning/i_maneuver_generator.h>
#include <motion_planning/i_trajectory_planner.h>
#include <motion_planning/i_trajectory_prioritizer.h>
#include <motion_planning/i_trajectory_selector.h>
#include <motion_planning/maneuver.h>
#include <motion_planning/maneuver_generator.h>
#include <motion_planning/trajectory_planner.h>
#include <motion_planning/trajectory_prioritizer.h>
#include <motion_planning/trajectory_selector.h>

namespace motion_planning
{
class MotionPlanning
{
  public:
    MotionPlanning();

    ~MotionPlanning() = default;

  private:
    std::unique_ptr<IManeuverGenerator> maneuver_generator_;
    std::unique_ptr<ITrajectoryPlanner> trajectory_planner_;
    std::unique_ptr<ITrajectoryPrioritizer> trajectory_prioritizer_;
    std::unique_ptr<ITrajectorySelector> trajectory_selector_;
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_MOTION_PLANNING_H_