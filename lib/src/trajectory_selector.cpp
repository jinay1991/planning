///
/// @file
///
#include <logging/logging.h>
#include <motion_planning/trajectory_selector.h>

namespace motion_planning
{
Trajectory TrajectorySelector::GetSelectedTrajectory(const PrioritizedTrajectories prioritized_trajectories) const
{
    const auto selected_trajectory = prioritized_trajectories.top();
    LOG_DEBUG("TrajectorySelector", selected_trajectory << std::endl);
    return selected_trajectory;
};

}  // namespace motion_planning
