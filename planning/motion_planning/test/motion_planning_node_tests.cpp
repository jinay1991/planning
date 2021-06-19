///
/// @file
/// @brief Contains component tests for MotionPlanning
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "middleware/communication/intra_process_pub_sub_factory.h"
#include "planning/motion_planning/node/motion_planning_node.h"
#include "planning/motion_planning/test/support/motion_planning_consumer_node.h"
#include "planning/motion_planning/test/support/motion_planning_simulator_node.h"

#include <gtest/gtest.h>
#include <units.h>

namespace planning
{
namespace
{
using namespace units::literals;

class MotionPlanningNodeFixture : public ::testing::Test
{
  public:
    MotionPlanningNodeFixture() : factory_{}, unit_{factory_}, consumer_{factory_}, simulator_{factory_} {}

  protected:
    void SetUp() override
    {
        simulator_.Init();
        unit_.Init();
        consumer_.Init();
    }

    void TearDown() override
    {
        simulator_.Shutdown();
        unit_.Shutdown();
        consumer_.Shutdown();
    }

    void RunOnce()
    {
        simulator_.Step();
        unit_.Step();
        consumer_.Step();
    }

    MotionPlanningSimulatorNode& GetSimulator() { return simulator_; }

    const Trajectory& GetSelectedTrajectory() const { return consumer_.GetSelectedTrajectory(); }

  private:
    middleware::IntraProcessPubSubFactory factory_;
    MotionPlanningNode unit_;
    MotionPlanningConsumerNode consumer_;
    MotionPlanningSimulatorNode simulator_;
};

TEST_F(MotionPlanningNodeFixture, Step_GivenTypicalInputs_ExpectSelectedTrajectory)
{
    // Given
    GetSimulator().BlockEgoLane();

    // When
    RunOnce();

    // Then
    const auto& selected_trajectory = GetSelectedTrajectory();
    EXPECT_GT(selected_trajectory.waypoints.size(), 0U);
    EXPECT_EQ(selected_trajectory.lane_id, LaneInformation::LaneId::kEgo);
    EXPECT_EQ(selected_trajectory.global_lane_id, LaneInformation::GlobalLaneId::kCenter);
}

}  // namespace
}  // namespace planning
