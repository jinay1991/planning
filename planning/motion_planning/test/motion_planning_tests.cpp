///
/// @file
/// @brief Contains component tests for Motion Planning.
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#include "planning/motion_planning/motion_planning.h"
#include "planning/motion_planning/roadmodel_data_source.h"
#include "planning/motion_planning/test/support/data_source_builder.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{
class MotionPlanningSpecFixture : public ::testing::Test
{
  public:
    MotionPlanningSpecFixture()
        : previous_path_global_{},
          map_waypoints_{
              MapCoordinates{GlobalCoordinates{784.6001, 1135.571}, FrenetCoordinates{0, 0, -0.02359831, -0.9997216}}},
          data_source_{
              DataSourceBuilder().WithPreviousPath(previous_path_global_).WithMapCoordinates(map_waypoints_).Build()},
          motion_planning_{data_source_}
    {
    }

  protected:
    void GenerateTrajectories() { motion_planning_.GenerateTrajectories(); }

    Trajectory GetSelectedTrajectory() const { return motion_planning_.GetSelectedTrajectory(); }

  private:
    const PreviousPathGlobal previous_path_global_;
    const std::vector<MapCoordinates> map_waypoints_;
    const IDataSource& data_source_;

    MotionPlanning motion_planning_;
};

TEST_F(MotionPlanningSpecFixture,
       DISABLED_ComponentTest_GivenTypicalInputs_WhenGenerateTrajectories_ThenReturnSelectedTrajectory)
{
    GenerateTrajectories();

    const auto actual = GetSelectedTrajectory();
    EXPECT_GT(actual.waypoints.size(), 0U);
    EXPECT_EQ(actual.lane_id, LaneInformation::LaneId::kEgo);
    EXPECT_EQ(actual.global_lane_id, LaneInformation::GlobalLaneId::kCenter);
}

// TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenGenerateManeuvers_ThenReturnThreeManeuvers) {}

// TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenPlanTrajectories_ThenReturnThreePlannedTrajectories) {}

// TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenEvaluatedTrajectories_ThenReturnEvalutatedTrajectories) {}

// TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenPrioritizedTrajectories_ThenReturnPrioritizedTrajectories) {}

// TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenSelectTrajectory_ThenReturnOnlyOneSelectedTrajectory) {}

}  // namespace
}  // namespace planning
