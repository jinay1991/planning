///
/// @file maneuver_generator.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_MANEUVER_GENERATOR_H_
#define MOTION_PLANNING_MANEUVER_GENERATOR_H_

#include <units.h>

#include "motion_planning/i_maneuver.h"
#include "motion_planning/i_maneuver_generator.h"
#include "motion_planning/maneuver.h"

namespace motion_planning
{
class ManeuverGenerator : public IManeuverGenerator
{
  public:
    virtual std::vector<Maneuver> Generate(const units::velocity::meters_per_second_t& target_velocity) const override;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_MANEUVER_GENERATOR_H_
