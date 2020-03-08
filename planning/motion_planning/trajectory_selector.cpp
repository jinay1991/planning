///
/// @file trajectory_selector.cpp
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include "planning/motion_planning/trajectory_selector.h"
#include "planning/common/logging/logging.h"

#include <algorithm>

namespace planning
{
Trajectory TrajectorySelector::GetSelectedTrajectory(const PrioritizedTrajectories& prioritized_trajectories) const
{
    const auto selected_trajectory = prioritized_trajectories.top();

    std::stringstream log_stream;
    log_stream << "Selected trajectory (lane_id): " << selected_trajectory.global_lane_id << std::endl;
    log_stream << " (+) " << selected_trajectory << std::endl;
    LOG(DEBUG) << log_stream.str();
    return selected_trajectory;
};

}  // namespace planning
