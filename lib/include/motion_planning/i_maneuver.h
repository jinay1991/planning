///
/// @file i_maneuver.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_I_MANEUVER_H_
#define MOTION_PLANNING_I_MANEUVER_H_

#include <units.h>

#include "motion_planning/domain_model/lane.h"

namespace motion_planning
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
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_MANEUVER_H_
