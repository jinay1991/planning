///
/// @file
/// @brief Contains unit tests for Trajectory Planner.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/test/support/builders/data_source_builder.h"
#include "planning/motion_planning/trajectory_planner.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

namespace planning
{
namespace
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using LaneId = LaneInformation::LaneId;

class TrajectoryPlannerFixture : public ::testing::TestWithParam<PreviousPathGlobal>
{
  protected:
    const std::vector<MapCoordinates> map_waypoints_{
        MapCoordinates{GlobalCoordinates{784.6001, 1135.571}, FrenetCoordinates{0, 0, -0.02359831, -0.9997216}}};
    const units::velocity::meters_per_second_t target_velocity_{10.0};
};

TEST_F(TrajectoryPlannerFixture, GetPlannedTrajectory_GivenInvalidManeuver_ExpectInvalidPlannedTrajectory)
{
    // Given
    const auto maneuvers = std::vector<Maneuver>{Maneuver{LaneId::kInvalid, target_velocity_}};
    const auto data_source =
        DataSourceBuilder().WithPreviousPath(PreviousPathGlobal{}).WithMapCoordinates(map_waypoints_).Build();

    // When
    const auto actual = TrajectoryPlanner(data_source).GetPlannedTrajectories(maneuvers);

    // Then
    EXPECT_EQ(actual.size(), maneuvers.size());
    EXPECT_EQ(actual[0].global_lane_id, GlobalLaneId::kInvalid);
}

INSTANTIATE_TEST_SUITE_P(
    TrajectoryPlanner,
    TrajectoryPlannerFixture,
    ::testing::Values(PreviousPathGlobal{},
                      PreviousPathGlobal{GlobalCoordinates{1, 2}, GlobalCoordinates{2, 2}},
                      PreviousPathGlobal{GlobalCoordinates{1, 2}, GlobalCoordinates{2, 2}, GlobalCoordinates{3, 2}}));

TEST_P(TrajectoryPlannerFixture, GivenTypicalManeuvers_ExpectPlannedTrajectories)
{
    // Given
    const auto maneuvers = std::vector<Maneuver>{Maneuver{LaneId::kEgo, target_velocity_}};
    const auto data_source =
        DataSourceBuilder().WithPreviousPath(GetParam()).WithMapCoordinates(map_waypoints_).Build();

    // When
    const auto actual = TrajectoryPlanner(data_source).GetPlannedTrajectories(maneuvers);

    // Then
    EXPECT_EQ(actual.size(), maneuvers.size());
    EXPECT_EQ(actual[0].global_lane_id, GlobalLaneId::kCenter);
}

}  // namespace
}  // namespace planning
