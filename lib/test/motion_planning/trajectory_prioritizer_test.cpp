///
/// @file
///

#include <gtest/gtest.h>
#include <motion_planning/trajectory_builder.h>
#include <motion_planning/trajectory_planner.h>
#include <motion_planning/trajectory_prioritizer.h>
#include <units.h>

using namespace motion_planning;

namespace
{
TEST(TrajectoryPrioritizerTest, GivenTypicalTrajectories_WhenPrioritized_ThenReturnedLeastCostTrajectory)
{
    const auto ego_trajectory = TrajectoryBuilder().WithLaneId(Maneuver::LaneId::kEgo).WithCost(1.0).Build();
    const auto left_trajectory = TrajectoryBuilder().WithLaneId(Maneuver::LaneId::kLeft).WithCost(2.0).Build();
    const auto right_trajectory = TrajectoryBuilder().WithLaneId(Maneuver::LaneId::kRight).WithCost(3.0).Build();

    const auto trajectories = PlannedTrajectories{ego_trajectory, left_trajectory, right_trajectory};

    auto actual = TrajectoryPrioritizer().GetPrioritizedTrajectories(trajectories);
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), Maneuver::LaneId::kEgo);
    EXPECT_DOUBLE_EQ(actual.top().cost, ego_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), Maneuver::LaneId::kLeft);
    EXPECT_DOUBLE_EQ(actual.top().cost, left_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), Maneuver::LaneId::kRight);
    EXPECT_DOUBLE_EQ(actual.top().cost, right_trajectory.cost);
}

TEST(TrajectoryPrioritizerTest,
     GivenTypicalTrajectories_WhenTwoTrajectoriesHaveSameCosts_ThenPrioritizedBasedOnLanePriority)
{
    const auto ego_trajectory = TrajectoryBuilder().WithLaneId(Maneuver::LaneId::kEgo).WithCost(3.0).Build();
    const auto left_trajectory = TrajectoryBuilder().WithLaneId(Maneuver::LaneId::kLeft).WithCost(3.0).Build();
    const auto right_trajectory = TrajectoryBuilder().WithLaneId(Maneuver::LaneId::kRight).WithCost(3.0).Build();

    const auto trajectories = PlannedTrajectories{ego_trajectory, left_trajectory, right_trajectory};

    auto actual = TrajectoryPrioritizer().GetPrioritizedTrajectories(trajectories);
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), Maneuver::LaneId::kRight);
    EXPECT_DOUBLE_EQ(actual.top().cost, right_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), Maneuver::LaneId::kEgo);
    EXPECT_DOUBLE_EQ(actual.top().cost, ego_trajectory.cost);

    actual.pop();
    EXPECT_EQ(actual.top().maneuver.GetLaneId(), Maneuver::LaneId::kLeft);
    EXPECT_DOUBLE_EQ(actual.top().cost, left_trajectory.cost);
}

}  // namespace
