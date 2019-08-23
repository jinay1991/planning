///
/// @file
///

#include <logging/logging.h>
#include <motion_planning/maneuver_generator.h>
#include <motion_planning/motion_planning.h>
#include <motion_planning/trajectory_planner.h>
#include <motion_planning/trajectory_prioritizer.h>
#include <motion_planning/trajectory_selector.h>

namespace motion_planning
{
MotionPlanning::MotionPlanning()
{
    maneuver_generator_ = std::make_unique<ManeuverGenerator>();
    trajectory_planner_ = std::make_unique<TrajectoryPlanner>();
    trajectory_prioritizer_ = std::make_unique<TrajectoryPrioritizer>();
    trajectory_selector_ = std::make_unique<TrajectorySelector>();
}
}  // namespace motion_planning
