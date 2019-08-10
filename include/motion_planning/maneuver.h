#ifndef MOTION_PLANNING_MANEUVER_H_
#define MOTION_PLANNING_MANEUVER_H_

#include <motion_planning/i_maneuver.h>
#include <units/units.h>

namespace motion_planning
{
class Maneuver : public IManeuver
{
  public:
    Maneuver();
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_MANEUVER_H_