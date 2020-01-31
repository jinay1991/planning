///
/// @file trajectory_selector.cpp
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include "motion_planning/trajectory_selector.h"
#include "logging/logging.h"

namespace motion_planning
{
Trajectory TrajectorySelector::GetSelectedTrajectory(const PrioritizedTrajectories& prioritized_trajectories) const
{
    const auto selected_trajectory = prioritized_trajectories.top();
    LOG(DEBUG) << "Selected trajectory (lane_id): " << selected_trajectory.global_lane_id;
    LOG(DEBUG) << " (+) " << selected_trajectory;
    return selected_trajectory;
};

}  // namespace motion_planning
