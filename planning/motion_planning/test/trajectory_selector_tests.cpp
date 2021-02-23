///
/// @file
/// @brief Contains unit tests for Trajectory Selector.
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#include "planning/motion_planning/test/support/trajectory_builder.h"
#include "planning/motion_planning/trajectory_prioritizer.h"
#include "planning/motion_planning/trajectory_selector.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{
TEST(TrajectorySelectorTest, GivenTypicalTrajectories_WhenPrioritized_ThenSelectedTopPriorityTrajectory)
{
    const auto ego_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kEgo).WithCost(1.0).Build();
    const auto left_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kLeft).WithCost(2.0).Build();
    const auto right_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kRight).WithCost(3.0).Build();

    const auto trajectories = Trajectories{ego_trajectory, left_trajectory, right_trajectory};
    const auto prioritized_trajectories = TrajectoryPrioritizer().GetPrioritizedTrajectories(trajectories);
    const auto actual = TrajectorySelector().GetSelectedTrajectory(prioritized_trajectories);

    EXPECT_EQ(actual.lane_id, LaneId::kEgo);
    EXPECT_DOUBLE_EQ(actual.cost, ego_trajectory.cost);
}

}  // namespace
}  // namespace planning
