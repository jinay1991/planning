///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_I_MANEUVER_GENERATOR_H
#define PLANNING_MOTION_PLANNING_I_MANEUVER_GENERATOR_H

#include "planning/motion_planning/maneuver.h"

#include <units.h>

#include <vector>

namespace planning
{
/// @brief Interface for Maneuver Generator
class IManeuverGenerator
{
  public:
    /// @brief Destructor
    virtual ~IManeuverGenerator() = default;

    /// @brief Generate Maneuvers (one for each lane, special maneuvers) with provided target velocity
    virtual std::vector<Maneuver> Generate(const units::velocity::meters_per_second_t target_velocity) const = 0;
};
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_I_MANEUVER_H
