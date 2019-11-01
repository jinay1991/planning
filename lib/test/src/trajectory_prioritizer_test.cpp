///
/// @file
///
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

#include "motion_planning/trajectory_planner.h"
#include "motion_planning/trajectory_prioritizer.h"

#include "trajectory_builder.h"

using namespace motion_planning;

namespace
{
TEST(TrajectoryPrioritizerTest, GivenTypicalTrajectories_WhenPrioritized_ThenReturnedLeastCostTrajectory)
{
    const auto ego_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kEgo).WithCost(1.0).Build();
    const auto left_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kLeft).WithCost(2.0).Build();
    const auto right_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kRight).WithCost(3.0).Build();

    const auto trajectories = PlannedTrajectories{ego_trajectory, left_trajectory, right_trajectory};

    auto actual = TrajectoryPrioritizer().GetPrioritizedTrajectories(trajectories);
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), LaneId::kEgo);
    EXPECT_DOUBLE_EQ(actual.top().cost, ego_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), LaneId::kLeft);
    EXPECT_DOUBLE_EQ(actual.top().cost, left_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), LaneId::kRight);
    EXPECT_DOUBLE_EQ(actual.top().cost, right_trajectory.cost);
}

TEST(TrajectoryPrioritizerTest,
     GivenTypicalTrajectories_WhenTwoTrajectoriesHaveSameCosts_ThenPrioritizedBasedOnLanePriority)
{
    const auto ego_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kEgo).WithCost(3.0).Build();
    const auto left_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kLeft).WithCost(3.0).Build();
    const auto right_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kRight).WithCost(3.0).Build();

    const auto trajectories = PlannedTrajectories{ego_trajectory, left_trajectory, right_trajectory};

    auto actual = TrajectoryPrioritizer().GetPrioritizedTrajectories(trajectories);
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), LaneId::kRight);
    EXPECT_DOUBLE_EQ(actual.top().cost, right_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), LaneId::kEgo);
    EXPECT_DOUBLE_EQ(actual.top().cost, ego_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), LaneId::kLeft);
    EXPECT_DOUBLE_EQ(actual.top().cost, left_trajectory.cost);
}

}  // namespace
