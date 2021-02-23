///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_I_VELOCITY_PLANNER_H
#define PLANNING_MOTION_PLANNING_I_VELOCITY_PLANNER_H

#include <units.h>

namespace planning
{
/// @brief Interface for Velocity Planner
class IVelocityPlanner
{
  public:
    /// @brief Destructor
    virtual ~IVelocityPlanner() = default;

    /// @brief Calculate Target Velocity based on the DataSource
    virtual void CalculateTargetVelocity() = 0;

    /// @brief Get Calculated Target Velocity
    virtual units::velocity::meters_per_second_t GetTargetVelocity() const = 0;
};
}  // namespace planning

#endif  // PLANNING_MOTION_PLANNING_I_VELOCITY_PLANNER_H
