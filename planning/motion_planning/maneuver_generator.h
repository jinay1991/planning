///
/// @file
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_MANEUVER_GENERATOR_H_
#define PLANNING_MOTION_PLANNING_MANEUVER_GENERATOR_H_

#include "planning/motion_planning/i_maneuver.h"
#include "planning/motion_planning/i_maneuver_generator.h"
#include "planning/motion_planning/maneuver.h"

#include <units.h>

namespace planning
{
/// @brief Maneuver Generator
class ManeuverGenerator : public IManeuverGenerator
{
  public:
    /// @brief Generate Maneuvers for given target velocity (one for each lane)
    virtual std::vector<Maneuver> Generate(const units::velocity::meters_per_second_t& target_velocity) const override;
};
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_MANEUVER_GENERATOR_H_
