///
/// @file maneuver_test.cpp
/// @brief Contains unit tests for Maneuver interfaces.
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include "planning/motion_planning/maneuver.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{
TEST(ManeuverSpecTest, HasEgoManeuverAsDefault)
{
    ASSERT_THAT(Maneuver(), ::testing::Eq(Maneuver(LaneId::kEgo, units::velocity::meters_per_second_t{0.0})));
}
}  // namespace
}  // namespace planning
