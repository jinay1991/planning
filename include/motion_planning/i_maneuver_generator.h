#ifndef MOTION_PLANNING_I_MANEUVER_H_
#define MOTION_PLANNING_I_MANEUVER_H_

#include <motion_planning/i_maneuver.h>

namespace motion_planning
{
class IManeuverGenerator
{
  public:
    virtual std::vector<IManeuver> Generate(const units::velocity::meters_per_second_t target_velocity) const = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_MANEUVER_H_