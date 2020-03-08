///
/// @file velocity_planner.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_VELOCITY_PLANNER_H_
#define PLANNING_MOTION_PLANNING_VELOCITY_PLANNER_H_

#include "planning/motion_planning/i_data_source.h"
#include "planning/motion_planning/i_velocity_planner.h"

#include <units.h>

#include <algorithm>
#include <iterator>
#include <memory>

namespace planning
{
/// @brief Velocity Planner
class VelocityPlanner : public IVelocityPlanner
{
  public:
    /// @brief Constructor. Initialize with DataSource.
    explicit VelocityPlanner(std::shared_ptr<IDataSource> data_source);

    /// @brief Constructor. Initialize with DataSource for given target velocity.
    explicit VelocityPlanner(std::shared_ptr<IDataSource> data_source,
                             const units::velocity::meters_per_second_t& target_velocity);

    /// @brief Calculate Target Velocity based on DataSource.
    virtual void CalculateTargetVelocity() override;

    /// @brief Get calculated target velocity.
    virtual units::velocity::meters_per_second_t GetTargetVelocity() const override;

  private:
    /// @brief Validate if vehicle/object in front (in same lane) within safe distance?
    virtual bool IsClosestInPathVehicleInFront(const ObjectFusion& object_fusion) const;

    /// @brief Get Delta Velocity between Ego and Object Velocity.
    virtual units::velocity::meters_per_second_t GetDeltaVelocity() const;

    /// @brief Vehicle Dynamics Refresh rate
    const units::frequency::hertz_t frequency_;

    /// @brief Jerk free deceleration rate
    const units::acceleration::meters_per_second_squared_t deceleration_;

    /// @brief Jerk free acceleration rate
    const units::acceleration::meters_per_second_squared_t acceleration_;

    /// @brief Target Velocity
    units::velocity::meters_per_second_t target_velocity_;

    /// @brief DataSource (contains information on VehicleDynamics, SensorFusion, Map Points etc.)
    std::shared_ptr<IDataSource> data_source_;
};
}  // namespace planning

#endif  // PLANNING_MOTION_PLANNING_VELOCITY_PLANNER_H_