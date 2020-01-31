///
/// @file velocity_planner.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_VELOCITY_PLANNER_H_
#define MOTION_PLANNING_VELOCITY_PLANNER_H_

#include <units.h>
#include <algorithm>
#include <iterator>
#include <memory>

#include "motion_planning/i_data_source.h"

namespace motion_planning
{
class VelocityPlanner
{
  public:
    explicit VelocityPlanner(std::shared_ptr<IDataSource> data_source);
    explicit VelocityPlanner(std::shared_ptr<IDataSource> data_source,
                             const units::velocity::meters_per_second_t& target_velocity);
    virtual ~VelocityPlanner() = default;

    virtual void CalculateTargetVelocity();

    virtual units::velocity::meters_per_second_t GetTargetVelocity() const;

  private:
    virtual bool IsClosestInPathVehicleInFront(const ObjectFusion& object_fusion) const;
    virtual units::velocity::meters_per_second_t GetDeltaVelocity() const;

    const units::frequency::hertz_t frequency_;
    const units::acceleration::meters_per_second_squared_t deceleration_;
    const units::acceleration::meters_per_second_squared_t acceleration_;

    units::velocity::meters_per_second_t target_velocity_;
    std::shared_ptr<IDataSource> data_source_;
};
}  // namespace motion_planning

#endif  // MOTION_PLANNING_VELOCITY_PLANNER_H_