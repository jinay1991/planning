///
/// @file
/// @brief Contains unit tests for Trajectory Evaluator.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/data_source.h"
#include "planning/motion_planning/test/support/builders/data_source_builder.h"
#include "planning/motion_planning/test/support/builders/trajectory_builder.h"
#include "planning/motion_planning/trajectory_evaluator.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

namespace planning
{
namespace
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using LaneId = LaneInformation::LaneId;

class TrajectoryEvaluatorFixture : public ::testing::Test
{
  public:
    TrajectoryEvaluatorFixture()
        : ego_global_lane_id_{GlobalLaneId::kCenter},
          target_velocity_{units::velocity::meters_per_second_t{18.0}},
          ego_trajectory_{TrajectoryBuilder()
                              .WithLaneId(LaneId::kEgo)
                              .WithGlobalLaneId(ego_global_lane_id_)
                              .WithTargetVelocity(target_velocity_)
                              .Build()},
          left_trajectory_{TrajectoryBuilder()
                               .WithLaneId(LaneId::kLeft)
                               .WithGlobalLaneId(ego_global_lane_id_ - 1)
                               .WithTargetVelocity(target_velocity_)
                               .Build()},
          right_trajectory_{TrajectoryBuilder()
                                .WithLaneId(LaneId::kRight)
                                .WithGlobalLaneId(ego_global_lane_id_ + 1)
                                .WithTargetVelocity(target_velocity_)
                                .Build()},
          planned_trajectories_{ego_trajectory_, left_trajectory_, right_trajectory_}
    {
    }

  protected:
    const GlobalLaneId ego_global_lane_id_;
    const units::velocity::meters_per_second_t target_velocity_;
    const Trajectory ego_trajectory_;
    const Trajectory left_trajectory_;
    const Trajectory right_trajectory_;
    const Trajectories planned_trajectories_;
};

TEST_F(TrajectoryEvaluatorFixture, GetRatedTrajectories_GivenTypicalPlannedTrajectories_ExpectSameNumberOfTrajectories)
{
    // Given
    const auto data_source = DataSourceBuilder().Build();

    // When
    const auto actual = TrajectoryEvaluator(data_source).GetRatedTrajectories(planned_trajectories_);

    // Then
    EXPECT_EQ(actual.size(), planned_trajectories_.size());
}

TEST_F(TrajectoryEvaluatorFixture,
       GetRatedTrajectories_GivenTypicalPlannedTrajectoriesWithNoObjects_ExpectSameCostTrajectories)
{
    // Given
    const auto data_source = DataSourceBuilder().Build();

    // When
    const auto actual = TrajectoryEvaluator(data_source).GetRatedTrajectories(planned_trajectories_);

    // Then
    const auto expected_cost = [&](const auto& trajectory)
    {
        const auto cost = (trajectory.lane_id != LaneId::kEgo) ? 1.0 : 0.0;
        EXPECT_DOUBLE_EQ(trajectory.cost, cost);
    };
    std::for_each(actual.begin(), actual.end(), expected_cost);
}

class TrajectoryEvaluatorFixture_WithEgoGlobalLaneId
    : public ::testing::TestWithParam<std::tuple<GlobalLaneId, GlobalLaneId>>
{
  protected:
    void SetUp() override
    {
        const auto ego_global_lane_id = std::get<0>(GetParam());

        const auto target_velocity = units::velocity::meters_per_second_t{18.0};
        const auto ego_trajectory = TrajectoryBuilder()
                                        .WithLaneId(LaneId::kEgo)
                                        .WithGlobalLaneId(ego_global_lane_id)
                                        .WithTargetVelocity(target_velocity)
                                        .Build();
        const auto left_trajectory = TrajectoryBuilder()
                                         .WithLaneId(LaneId::kLeft)
                                         .WithGlobalLaneId(ego_global_lane_id - 1)
                                         .WithTargetVelocity(target_velocity)
                                         .Build();
        const auto right_trajectory = TrajectoryBuilder()
                                          .WithLaneId(LaneId::kRight)
                                          .WithGlobalLaneId(ego_global_lane_id + 1)
                                          .WithTargetVelocity(target_velocity)
                                          .Build();
        planned_trajectories_ = Trajectories{ego_trajectory, left_trajectory, right_trajectory};
    }

    Trajectories planned_trajectories_{};
};

INSTANTIATE_TEST_SUITE_P(
    TrajectoryEvaluator,
    TrajectoryEvaluatorFixture_WithEgoGlobalLaneId,
    ::testing::Combine(::testing::Values(GlobalLaneId::kLeft, GlobalLaneId::kCenter, GlobalLaneId::kRight),
                       ::testing::Values(GlobalLaneId::kLeft, GlobalLaneId::kCenter, GlobalLaneId::kRight)));

TEST_P(TrajectoryEvaluatorFixture_WithEgoGlobalLaneId,
       GetRatedTrajectories_GivenTypicalPlannedTrajectoriesWithObjectInLane_ExpectHighCostToTrajectoryOnThatLane)
{
    // Given
    const auto ego_global_lane_id = std::get<0>(GetParam());
    const auto obj_global_lane_id = std::get<1>(GetParam());
    const auto ego_velocity = units::velocity::meters_per_second_t{10.0};
    const auto data_source = DataSourceBuilder()
                                 .WithGlobalLaneId(ego_global_lane_id)
                                 .WithObjectInLane(obj_global_lane_id, ego_velocity)
                                 .Build();

    // When
    const auto actual = TrajectoryEvaluator(data_source).GetRatedTrajectories(planned_trajectories_);

    // Then
    const auto expected_cost = [&](const auto& trajectory)
    {
        if (trajectory.global_lane_id == GlobalLaneId::kInvalid || trajectory.global_lane_id == obj_global_lane_id)
        {
            EXPECT_DOUBLE_EQ(trajectory.cost, std::numeric_limits<double>::infinity());
        }
        else
        {
            const auto cost = (trajectory.lane_id != LaneId::kEgo) ? 1.0 : 0.0;
            EXPECT_DOUBLE_EQ(trajectory.cost, cost);
        }
    };
    std::for_each(actual.begin(), actual.end(), expected_cost);
}

}  // namespace
}  // namespace planning
