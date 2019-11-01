///
/// @file
///
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "motion_planning/trajectory_prioritizer.h"
#include "motion_planning/trajectory_selector.h"

#include "trajectory_builder.h"

using namespace motion_planning;

namespace
{
TEST(TrajectorySelectorTest, GivenTypicalTrajectories_WhenPrioritized_ThenSelectedTopPriorityTrajectory)
{
    const auto ego_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kEgo).WithCost(1.0).Build();
    const auto left_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kLeft).WithCost(2.0).Build();
    const auto right_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kRight).WithCost(3.0).Build();

    const auto trajectories = PlannedTrajectories{ego_trajectory, left_trajectory, right_trajectory};
    const auto prioritized_trajectories = TrajectoryPrioritizer().GetPrioritizedTrajectories(trajectories);
    const auto actual = TrajectorySelector().GetSelectedTrajectory(prioritized_trajectories);

    EXPECT_EQ(actual.maneuver.GetLaneId(), LaneId::kEgo);
    EXPECT_DOUBLE_EQ(actual.cost, ego_trajectory.cost);
}

}  // namespace
