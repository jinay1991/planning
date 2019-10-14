///
/// @file
///

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <motion_planning/maneuver_generator.h>

namespace motion_planning
{
namespace
{
TEST(ManeuverGeneratorSpecTest, GeneratesThreeManeuvers)
{
    const auto target_velocity = units::velocity::meters_per_second_t{17.0};
    const auto maneuvers = ManeuverGenerator().Generate(target_velocity);
    ASSERT_THAT(maneuvers.size(), ::testing::Eq(3U));
}

}  // namespace
}  // namespace motion_planning
