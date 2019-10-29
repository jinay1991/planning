///
/// @file
///

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <motion_planning/maneuver.h>

using namespace motion_planning;

namespace
{
TEST(ManeuverSpecTest, HasEgoManeuverAsDefault)
{
    ASSERT_THAT(Maneuver(), ::testing::Eq(Maneuver(LaneId::kEgo, units::velocity::meters_per_second_t{0.0})));
}

}  // namespace
