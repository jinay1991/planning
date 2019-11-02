///
/// @file
///
#include "motion_planning/velocity_planner/velocity_planner.h"

namespace motion_planning
{
VelocityPlanner::VelocityPlanner(std::shared_ptr<IDataSource> data_source) : data_source_{data_source} {}

bool VelocityPlanner::IsClosestInPathVehicleInFront(const ObjectFusion& object_fusion) const
{
    const auto ego_lane_id = data_source_->GetGlobalLaneId();
    const auto ego_position = data_source_->GetPreviousPathEnd();
    const auto obj_position = object_fusion.frenet_coords;
    const auto obj_lane_id = data_source_->GetGlobalLaneId(object_fusion.frenet_coords);
    const auto is_near = (std::fabs(obj_position.s - ego_position.s) < gkFarDistanceThreshold.value());

    if (is_near && (obj_lane_id == ego_lane_id))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void VelocityPlanner::CalculateTargetVelocity()
{
    const auto vehicle_dynamics = data_source_->GetVehicleDynamics();
    const auto sensor_fusion = data_source_->GetSensorFusion();
    const auto frequency = units::frequency::hertz_t{25.0};

    target_velocity_ = vehicle_dynamics.velocity;
    const auto adjust_velocity = [&](const auto& obj) {
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
    };
    std::for_each(sensor_fusion.objs.begin(), sensor_fusion.objs.end(), adjust_velocity);
}

units::velocity::meters_per_second_t VelocityPlanner::GetTargetVelocity() const { return target_velocity_; }

}  // namespace motion_planning