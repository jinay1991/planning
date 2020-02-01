///
/// @file trajectory_selector.cpp
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include <algorithm>

#include "logging/logging.h"
#include "motion_planning/trajectory_selector.h"

namespace motion_planning
{
Trajectory TrajectorySelector::GetSelectedTrajectory(const PrioritizedTrajectories& prioritized_trajectories) const
{
    const auto selected_trajectory = prioritized_trajectories.top();

    std::stringstream log_stream;
    log_stream << "Selected trajectory (lane_id): " << selected_trajectory.global_lane_id << std::endl;
    log_stream << " (+) " << selected_trajectory << std::endl;
    std::for_each(selected_trajectory.waypoints.begin(), selected_trajectory.waypoints.begin() + 3,
                  [&log_stream](const auto& wp) { log_stream << "     => " << wp << std::endl; });
    log_stream << "     => ... (more " << selected_trajectory.waypoints.size() - 3 << ") waypoints." << std::endl;
    LOG(DEBUG) << log_stream.str();
    return selected_trajectory;
};

}  // namespace motion_planning
