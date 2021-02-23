///
/// @file
/// @brief Contains component tests for Motion Planning.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/data_source.h"
#include "planning/motion_planning/motion_planning.h"
#include "planning/motion_planning/test/support/data_source_builder.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{
class MotionPlanningFixture : public ::testing::Test
{
  public:
    MotionPlanningFixture()
        : previous_path_global_{},
          map_waypoints_{
              MapCoordinates{GlobalCoordinates{784.6001, 1135.571}, FrenetCoordinates{0, 0, -0.02359831, -0.9997216}}}
    {
    }

  protected:
    const PreviousPathGlobal previous_path_global_;
    const std::vector<MapCoordinates> map_waypoints_;
};

TEST_F(MotionPlanningFixture, DISABLED_GenerateTrajectories_GivenTypicalDataSource_ExpectSelectedTrajectory)
{
    // Given
    const auto data_source =
        DataSourceBuilder().WithPreviousPath(previous_path_global_).WithMapCoordinates(map_waypoints_).Build();
    auto motion_planning = MotionPlanning(data_source);

    // When
    motion_planning.GenerateTrajectories();

    // Then
    const auto actual = motion_planning.GetSelectedTrajectory();
    EXPECT_GT(actual.waypoints.size(), 0U);
    EXPECT_EQ(actual.lane_id, LaneInformation::LaneId::kEgo);
    EXPECT_EQ(actual.global_lane_id, LaneInformation::GlobalLaneId::kCenter);
}

}  // namespace
}  // namespace planning
