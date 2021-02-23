///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_I_MANEUVER_H
#define PLANNING_MOTION_PLANNING_I_MANEUVER_H

#include "planning/datatypes/lane.h"

#include <units.h>

namespace planning
{
using LaneId = LaneInformation::LaneId;

/// @brief Interface for Maneuver
class IManeuver
{
  public:
    /// @brief Destructor
    virtual ~IManeuver() = default;

    /// @brief Get LaneId in Local Coordinates System for the Maneuver
    virtual LaneId GetLaneId() const = 0;

    /// @brief Get Target Velocity for the Maneuver
    virtual units::velocity::meters_per_second_t GetVelocity() const = 0;
};
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_I_MANEUVER_H
