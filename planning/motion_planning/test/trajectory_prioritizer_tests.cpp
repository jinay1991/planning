///
/// @file
/// @brief Contains unit tests for Trajectory Prioritizer.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///

#include "planning/motion_planning/test/support/trajectory_builder.h"
#include "planning/motion_planning/trajectory_planner.h"
#include "planning/motion_planning/trajectory_prioritizer.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

namespace planning
{
namespace
{
using ::testing::AllOf;
using ::testing::ElementsAre;
using ::testing::Field;

TEST(TrajectoryPrioritizerTest, GetPrioritizedTrajectories_GivenTypicalTrajectories_ExpectedLeastCostTrajectory)
{
    // Given
    const auto ego_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kEgo).WithCost(1.0).Build();
    const auto left_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kLeft).WithCost(2.0).Build();
    const auto right_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kRight).WithCost(3.0).Build();
    const auto trajectories = Trajectories{ego_trajectory, left_trajectory, right_trajectory};

    // When
    auto actual = TrajectoryPrioritizer().GetPrioritizedTrajectories(trajectories);

    // Then
    EXPECT_EQ(actual.top().lane_id, LaneId::kEgo);
    EXPECT_DOUBLE_EQ(actual.top().cost, ego_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().lane_id, LaneId::kLeft);
    EXPECT_DOUBLE_EQ(actual.top().cost, left_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().lane_id, LaneId::kRight);
    EXPECT_DOUBLE_EQ(actual.top().cost, right_trajectory.cost);
}

TEST(TrajectoryPrioritizerTest,
     GetPrioritizedTrajectories_GivenTwoTrajectoriesHaveSameCosts_ExpectPrioritizedBasedOnLanePriority)
{
    // Given
    const auto ego_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kEgo).WithCost(3.0).Build();
    const auto left_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kLeft).WithCost(3.0).Build();
    const auto right_trajectory = TrajectoryBuilder().WithLaneId(LaneId::kRight).WithCost(3.0).Build();
    const auto trajectories = Trajectories{ego_trajectory, left_trajectory, right_trajectory};

    // When
    auto actual = TrajectoryPrioritizer().GetPrioritizedTrajectories(trajectories);

    // Then
    EXPECT_EQ(actual.top().lane_id, LaneId::kRight);
    EXPECT_DOUBLE_EQ(actual.top().cost, right_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().lane_id, LaneId::kEgo);
    EXPECT_DOUBLE_EQ(actual.top().cost, ego_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().lane_id, LaneId::kLeft);
    EXPECT_DOUBLE_EQ(actual.top().cost, left_trajectory.cost);
}

}  // namespace
}  // namespace planning
