///
/// @file
///

#include <motion_planning/velocity_planner/velocity_planner.h>

namespace motion_planning
{
VelocityPlanner::VelocityPlanner(std::shared_ptr<IDataSource> data_source) : data_source_{data_source} {}

bool VelocityPlanner::IsClosestInPathVehicleInFront(const ObjectFusion& object_fusion) const { return true; }

void VelocityPlanner::CalculateTargetVelocity()
{
    const auto vehicle_dynamics = data_source_->GetVehicleDynamics();
    const auto sensor_fusion = data_source_->GetSensorFusion();
    const auto frequency = units::frequency::hertz_t{25.0};

    target_velocity_ = vehicle_dynamics.velocity;
    std::for_each(sensor_fusion.objs.begin(), sensor_fusion.objs.end(), [&](const auto& obj) {
        if (target_velocity_ < max_allowed_velocity_)
        {
            if (IsClosestInPathVehicleInFront(obj))
            {
                target_velocity_ += (deceleration_ / frequency);
            }
            else
            {
                target_velocity_ += (acceleration_ / frequency);
            }
        }
    });

    // target_velocity_ = units::velocity::meters_per_second_t{20.00};
}

units::velocity::meters_per_second_t VelocityPlanner::GetTargetVelocity() const { return target_velocity_; }

}  // namespace motion_planning