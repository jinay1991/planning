///
/// @file
///
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

#include "motion_planning/trajectory_planner.h"

#include "data_source_builder.h"

namespace motion_planning
{
namespace
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using LaneId = LaneInformation::LaneId;

class TrajectoryPlannerSpec : public ::testing::Test
{
  public:
    virtual void SetUp() override
    {
        const auto previous_path_global = PreviousPathGlobal{};
        const auto map_waypoints = std::vector<MapCoordinates>{
            MapCoordinates{GlobalCoordinates{784.6001, 1135.571}, FrenetCoordinates{0, 0, -0.02359831, -0.9997216}}};
        auto data_source =
            DataSourceBuilder().WithPreviousPath(previous_path_global).WithMapCoordinates(map_waypoints).Build();

        trajectory_planner_ = std::make_unique<TrajectoryPlanner>(data_source);
    }

  protected:
    std::unique_ptr<ITrajectoryPlanner> trajectory_planner_;
    const std::vector<Maneuver> maneuvers_{Maneuver{LaneId::kEgo, units::velocity::meters_per_second_t{10.0}}};
};
TEST_F(TrajectoryPlannerSpec, GivenTypicalManeuvers_WhenEvaluated_ThenReturnSameNumberOfPlannedTrajectories)
{
    const auto actual = trajectory_planner_->GetPlannedTrajectories(maneuvers_);

    EXPECT_EQ(actual.size(), maneuvers_.size());
}
}  // namespace 
}  // namespace motion_planning
