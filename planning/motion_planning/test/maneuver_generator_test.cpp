///
/// @file maneuver_generator_test.cpp
/// @brief Contains unit tests for Maneuver Generation.
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include "planning/motion_planning/maneuver_generator.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{
TEST(ManeuverGeneratorSpecTest, GeneratesThreeManeuvers)
{
    const auto target_velocity = units::velocity::meters_per_second_t{17.0};
    const auto maneuvers = ManeuverGenerator().Generate(target_velocity);
    ASSERT_THAT(maneuvers.size(), ::testing::Eq(3U));
    EXPECT_EQ(maneuvers[0].GetLaneId(), LaneId::kLeft);
    EXPECT_EQ(maneuvers[0].GetVelocity(), target_velocity);
    EXPECT_EQ(maneuvers[1].GetLaneId(), LaneId::kEgo);
    EXPECT_EQ(maneuvers[1].GetVelocity(), target_velocity);
    EXPECT_EQ(maneuvers[2].GetLaneId(), LaneId::kRight);
    EXPECT_EQ(maneuvers[2].GetVelocity(), target_velocity);
}

}  // namespace
}  // namespace planning
