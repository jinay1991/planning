///
/// @file
///

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <motion_planning/maneuver.h>
namespace motion_planning
{
namespace
{
TEST(ManeuverTest, HasEgoManeuverAsDefault)
{
    ASSERT_THAT(Maneuver(), ::testing::Eq(Maneuver(Maneuver::LaneId::kEgo, units::velocity::meters_per_second_t{0.0})));
}

}  // namespace
}  // namespace motion_planning