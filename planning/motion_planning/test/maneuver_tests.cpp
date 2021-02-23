///
/// @file
/// @brief Contains unit tests for Maneuver interfaces.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/maneuver.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

namespace planning
{
namespace
{
TEST(ManeuverTest, HasEgoManeuverAsDefault)
{
    // Then
    ASSERT_THAT(Maneuver(), ::testing::Eq(Maneuver(LaneId::kEgo, units::velocity::meters_per_second_t{0.0})));
}

TEST(ManeuverTest, NonDefaultConstruction)
{
    // When
    const auto unit = Maneuver(LaneId::kRight, units::velocity::meters_per_second_t{10.0});

    // Then
    EXPECT_EQ(unit.GetLaneId(), LaneId::kRight);
    EXPECT_THAT(unit.GetVelocity(), ::testing::Eq(units::velocity::meters_per_second_t{10.0}));
}

TEST(ManeuverTest, CompareManeuvers)
{
    // Then
    EXPECT_EQ(Maneuver(), Maneuver(LaneId::kEgo, units::velocity::meters_per_second_t{0.0}));
}

}  // namespace
}  // namespace planning
