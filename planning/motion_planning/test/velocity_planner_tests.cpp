///
/// @file
/// @brief Contains unit tests for Velocity Planner.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/datatypes/lane.h"
#include "planning/motion_planning/test/support/builders/data_source_builder.h"
#include "planning/motion_planning/velocity_planner.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{
using namespace units::literals;

using LaneId = LaneInformation::LaneId;
using GlobalLaneId = LaneInformation::GlobalLaneId;

class VelocityPlannerFixture : public ::testing::Test
{
  public:
    VelocityPlannerFixture() : velocity_{17.0} {}

  protected:
    const units::velocity::meters_per_second_t velocity_;
};

template <typename T>
class VelocityPlannerFixtureT : public VelocityPlannerFixture, public ::testing::WithParamInterface<T>
{
};

TEST(VelocityPlannerTest, Constructor_GivenTypicalDataSource_ExpectInitialValues)
{
    // Given
    const auto data_source = DataSourceBuilder().Build();

    // When
    const VelocityPlanner velocity_planner{data_source};

    // Then
    EXPECT_EQ(velocity_planner.GetTargetVelocity(), 0.0_mps);
}

TEST(VelocityPlannerTest, Constructor_GivenTypicalDataSourceWithTargetVelocity_ExpectSameTargetVelocity)
{
    // Given
    const auto data_source = DataSourceBuilder().Build();
    const auto target_velocity = 10.0_mps;

    // When
    const VelocityPlanner velocity_planner{data_source, target_velocity};

    // Then
    EXPECT_EQ(velocity_planner.GetTargetVelocity(), target_velocity);
}

using VelocityPlannerFixture_WithAcceleration =
    VelocityPlannerFixtureT<std::tuple<units::velocity::meters_per_second_t, GlobalLaneId>>;

INSTANTIATE_TEST_SUITE_P(VelocityPlanner,
                         VelocityPlannerFixture_WithAcceleration,
                         ::testing::Combine(::testing::Values(0.0_mps, 10.0_mps, 17.0_mps, 20.0_mps),
                                            ::testing::Values(GlobalLaneId::kLeft, GlobalLaneId::kRight)));

TEST_P(VelocityPlannerFixture_WithAcceleration,
       CalculateTargetVelocity_GivenNoClosestInPathVehicle_ExpectAcceleratedTargetVelocity)
{
    // Given
    const auto object_velocity = std::get<0>(GetParam());
    const auto object_lane_id = std::get<1>(GetParam());
    auto data_source = DataSourceBuilder()
                           .WithVelocity(velocity_)
                           .WithGlobalLaneId(GlobalLaneId::kCenter)
                           .WithDistance(0.0_m)
                           .WithObjectInLane(object_lane_id, object_velocity)
                           .Build();
    VelocityPlanner velocity_planner{data_source, velocity_};

    // When
    velocity_planner.CalculateTargetVelocity();

    // Then
    EXPECT_GT(velocity_planner.GetTargetVelocity(), velocity_);
}

using VelocityPlannerFixture_WithDeceleration =
    VelocityPlannerFixtureT<std::tuple<units::velocity::meters_per_second_t, GlobalLaneId>>;

INSTANTIATE_TEST_SUITE_P(VelocityPlanner,
                         VelocityPlannerFixture_WithDeceleration,
                         ::testing::Combine(::testing::Values(0.0_mps, 15.0_mps, 17.0_mps),
                                            ::testing::Values(GlobalLaneId::kCenter)));

TEST_P(VelocityPlannerFixture_WithDeceleration,
       CalculateTargetVelocity_GivenClosestInPathVehicle_ExpectDeceleratedTargetVelocity)
{
    // Given
    const auto object_velocity = std::get<0>(GetParam());
    const auto object_lane_id = std::get<1>(GetParam());
    const auto data_source = DataSourceBuilder()
                                 .WithVelocity(velocity_)
                                 .WithGlobalLaneId(GlobalLaneId::kCenter)
                                 .WithDistance(0.0_m)
                                 .WithObjectInLane(object_lane_id, object_velocity)
                                 .Build();
    VelocityPlanner velocity_planner{data_source, velocity_};

    // When
    velocity_planner.CalculateTargetVelocity();

    // Then
    EXPECT_LT(velocity_planner.GetTargetVelocity(), velocity_);
}

}  // namespace
}  // namespace planning
