///
/// @file
///

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

#include "motion_planning/roadmodel_data_source.h"
#include "motion_planning/trajectory_evaluator.h"

#include "data_source_builder.h"
#include "trajectory_builder.h"

using namespace motion_planning;

namespace
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using LaneId = LaneInformation::LaneId;

class TrajectoryEvaluatorSpec : public ::testing::Test
{
  public:
    TrajectoryEvaluatorSpec()
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
    const PlannedTrajectories planned_trajectories_;
};
TEST_F(TrajectoryEvaluatorSpec, GivenTypicalPlannedTrajectories_WhenEvaluated_ThenReturnSameNumberOfTrajectories)
{
    const auto actual = TrajectoryEvaluator(DataSourceBuilder().Build()).GetRatedTrajectories(planned_trajectories_);

    EXPECT_EQ(actual.size(), planned_trajectories_.size());
}

TEST_F(TrajectoryEvaluatorSpec, DISABLED_GivenTypicalPlannedTrajectories_WhenNoObject_ThenReturnSameCostTrajectories)
{
    const auto actual = TrajectoryEvaluator(DataSourceBuilder().Build()).GetRatedTrajectories(planned_trajectories_);

    const auto expect_zero_cost = [&](const auto& trajectory) { EXPECT_DOUBLE_EQ(trajectory.cost, 0.0); };
    std::for_each(actual.begin(), actual.end(), expect_zero_cost);
}

class TrajectoryEvaluatorSpecFixture : public ::testing::TestWithParam<std::tuple<GlobalLaneId, GlobalLaneId>>
{
  protected:
    virtual void SetUp() override
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
        planned_trajectories_ = PlannedTrajectories{ego_trajectory, left_trajectory, right_trajectory};
    }

    PlannedTrajectories planned_trajectories_{};
};

TEST_P(TrajectoryEvaluatorSpecFixture,
       DISABLED_GivenTypicalPlannedTrajectories_WhenObjectInLane_ThenReturnHighCostToTrajectoryOnThatLane)
{
    const auto ego_global_lane_id = std::get<0>(GetParam());
    const auto obj_global_lane_id = std::get<1>(GetParam());
    const auto actual =
        TrajectoryEvaluator(DataSourceBuilder()
                                .WithGlobalLaneId(ego_global_lane_id)
                                .WithObjectInLane(obj_global_lane_id, units::velocity::meters_per_second_t{10.0})
                                .Build())
            .GetRatedTrajectories(planned_trajectories_);

    const auto expect_cost = [&](const auto& trajectory) {
        if (trajectory.global_lane_id == GlobalLaneId::kInvalid || trajectory.global_lane_id == obj_global_lane_id)
        {
            EXPECT_DOUBLE_EQ(trajectory.cost, std::numeric_limits<double>::infinity());
        }
        else
        {
            EXPECT_DOUBLE_EQ(trajectory.cost, 0.0);
        }
    };
    std::for_each(actual.begin(), actual.end(), expect_cost);
}
INSTANTIATE_TEST_CASE_P(
    TrajectoryEvaluator, TrajectoryEvaluatorSpecFixture,
    ::testing::Combine(::testing::Values(GlobalLaneId::kLeft, GlobalLaneId::kCenter, GlobalLaneId::kRight),
                       ::testing::Values(GlobalLaneId::kLeft, GlobalLaneId::kCenter, GlobalLaneId::kRight)));

}  // namespace
