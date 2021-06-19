///
/// @file
/// @brief Contains unit tests for Trajectory Optimizer.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/test/support/builders/data_source_builder.h"
#include "planning/motion_planning/test/support/builders/trajectory_builder.h"
#include "planning/motion_planning/trajectory_optimizer.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

namespace planning
{
namespace
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using LaneId = LaneInformation::LaneId;

class TrajectoryOptimizerFixture : public ::testing::Test
{
  protected:
    const std::vector<MapCoordinates> map_waypoints_{
        MapCoordinates{GlobalCoordinates{784.6001, 1135.571}, FrenetCoordinates{0, 0, -0.02359831, -0.9997216}}};
    const DataSource data_source_{
        DataSourceBuilder().WithPreviousPath(PreviousPathGlobal{}).WithMapCoordinates(map_waypoints_).Build()};

    TrajectoryOptimizer trajectory_optimizer_{data_source_};
};

TEST_F(TrajectoryOptimizerFixture, GetOptimizedTrajectories_GivenPlannedTrajectories_ExpectOptimizedTrajectory)
{
    // Given
    const auto start_position = GlobalCoordinates{0, 0};
    const auto start_yaw = units::angle::radian_t{0.0};
    const auto displacement = units::length::meter_t{30.0};
    const std::int32_t count = 30;
    const auto trajectories =
        Trajectories{TrajectoryBuilder().WithWaypoints(start_position, start_yaw, count, displacement).Build()};

    // When
    const auto actual = trajectory_optimizer_.GetOptimizedTrajectories(trajectories);

    // Then
    EXPECT_EQ(actual.size(), trajectories.size());
    EXPECT_EQ(actual[0].global_lane_id, GlobalLaneId::kCenter);
}

/// @todo Add Test confirming the curve for Trajectory is smooth enough

}  // namespace
}  // namespace planning
