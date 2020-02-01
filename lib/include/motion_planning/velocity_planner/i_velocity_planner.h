///
/// @file i_velocity_planner.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_I_VELOCITY_PLANNER_H_
#define MOTION_PLANNING_I_VELOCITY_PLANNER_H_

#include <units.h>

namespace motion_planning
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
}  // namespace motion_planning

#endif  // MOTION_PLANNING_I_VELOCITY_PLANNER_H_