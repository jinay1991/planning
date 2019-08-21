///
/// @file
///
#ifndef MOTION_PLANNING_MANEUVER_GENERATOR_H_
#define MOTION_PLANNING_MANEUVER_GENERATOR_H_

#include <motion_planning/i_maneuver.h>
#include <units/units.h>

namespace motion_planning
{
class ManeuverGenerator : public IManeuverGenerator
{
  public:
    ManeuverGenerator();

    std::vector<IManeuver> Generate(const units::velocity::meters_per_second_t target_velocity) const override;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_MANEUVER_GENERATOR_H_