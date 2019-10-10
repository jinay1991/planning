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
    auto maneuver_generator = ManeuverGenerator{};
    auto maneuvers = maneuver_generator.Generate(units::velocity::meters_per_second_t{17.0});
    ASSERT_THAT(maneuvers.size(), ::testing::Eq(3U));
}

}  // namespace
}  // namespace motion_planning
