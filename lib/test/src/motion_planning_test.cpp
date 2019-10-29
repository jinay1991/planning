///
/// @file
///
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <motion_planning/motion_planning.h>
#include <motion_planning/roadmodel_data_source.h>

#include "data_source_builder.h"

using namespace motion_planning;

namespace
{
class MotionPlanningSpecFixture : public ::testing::Test
{
  protected:
    virtual void SetUp() override
    {
        const auto previous_path_global = PreviousPathGlobal{};
        const auto map_waypoints = std::vector<MapCoordinates>{
            MapCoordinates{GlobalCoordinates{784.6001, 1135.571}, FrenetCoordinates{0, 0, -0.02359831, -0.9997216}}};
        data_source_ =
            DataSourceBuilder().WithPreviousPath(previous_path_global).WithMapCoordinates(map_waypoints).Build();

        motion_planning_ = std::make_unique<MotionPlanning>(data_source_);
    }
    virtual void TearDown() override {}

    std::unique_ptr<MotionPlanning> motion_planning_;

  private:
    std::shared_ptr<IDataSource> data_source_;
};

TEST_F(MotionPlanningSpecFixture, GivenTypicalInputs_WhenGenerateTrajectories_ThenReturnSelectedTrajectory)
{
    motion_planning_->GenerateTrajectories();

    const auto actual = motion_planning_->GetSelectedTrajectory();
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
