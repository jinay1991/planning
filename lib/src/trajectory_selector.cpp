///
/// @file
///
#include "motion_planning/trajectory_selector.h"
#include "logging/logging.h"

namespace motion_planning
{
Trajectory TrajectorySelector::GetSelectedTrajectory(const PrioritizedTrajectories& prioritized_trajectories) const
{
    const auto selected_trajectory = prioritized_trajectories.top();
    std::stringstream log_stream;
    log_stream << "Selected trajectory (lane_id): " << selected_trajectory.global_lane_id << std::endl;
    log_stream << " (+) " << selected_trajectory << std::endl;
    LOG_DEBUG("TrajectorySelector", log_stream.str());
    return selected_trajectory;
};

}  // namespace motion_planning
