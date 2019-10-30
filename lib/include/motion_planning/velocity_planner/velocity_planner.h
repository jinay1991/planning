///
/// @file
///
#ifndef MOTION_PLANNING_VELOCITY_PLANNER_H_
#define MOTION_PLANNING_VELOCITY_PLANNER_H_

#include <units.h>
#include <memory>

#include <motion_planning/i_data_source.h>

namespace motion_planning
{
class VelocityPlanner
{
  public:
    explicit VelocityPlanner(std::shared_ptr<IDataSource> data_source);
    virtual ~VelocityPlanner() = default;

    void CalculateTargetVelocity();

    units::velocity::meters_per_second_t GetTargetVelocity() const;

  private:
    bool IsClosestInPathVehicleInFront(const ObjectFusion& object_fusion) const;

    units::velocity::meters_per_second_t target_velocity_{0.0};
    units::velocity::meters_per_second_t max_allowed_velocity_{23.0};
    std::shared_ptr<IDataSource> data_source_;

    const units::acceleration::meters_per_second_squared_t deceleration_{-5.0};
    const units::acceleration::meters_per_second_squared_t acceleration_{5.0};
};
}  // namespace motion_planning

#endif  // MOTION_PLANNING_VELOCITY_PLANNER_H_