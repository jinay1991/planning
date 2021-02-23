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
TEST(ManeuverSpecTest, HasEgoManeuverAsDefault)
{
    ASSERT_THAT(Maneuver(), ::testing::Eq(Maneuver(LaneId::kEgo, units::velocity::meters_per_second_t{0.0})));
}

TEST(ManeuverSpecTest, NonDefaultConstruction)
{
    const auto unit = Maneuver(LaneId::kRight, units::velocity::meters_per_second_t{10.0});
    EXPECT_EQ(unit.GetLaneId(), LaneId::kRight);
    EXPECT_THAT(unit.GetVelocity(), ::testing::Eq(units::velocity::meters_per_second_t{10.0}));
}

TEST(ManeuverSpecTest, CompareManeuvers)
{
    EXPECT_EQ(Maneuver(), Maneuver(LaneId::kEgo, units::velocity::meters_per_second_t{0.0}));
}

}  // namespace
}  // namespace planning
