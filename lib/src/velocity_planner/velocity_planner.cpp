///
/// @file
///
#include <logging/logging.h>

#include "motion_planning/velocity_planner/velocity_planner.h"

namespace motion_planning
{
VelocityPlanner::VelocityPlanner(std::shared_ptr<IDataSource> data_source)
    : VelocityPlanner{data_source, units::velocity::meters_per_second_t{0.0}}
{
}

VelocityPlanner::VelocityPlanner(std::shared_ptr<IDataSource> data_source,
                                 const units::velocity::meters_per_second_t& target_velocity)
    : frequency_{22.0},
      deceleration_{-5.0},
      acceleration_{5.0},
      target_velocity_{target_velocity},
      data_source_{data_source}
{
}

bool VelocityPlanner::IsClosestInPathVehicleInFront(const ObjectFusion& object_fusion) const
{
    const auto ego_lane_id = data_source_->GetGlobalLaneId();
    const auto ego_position = data_source_->GetPreviousPathEnd();
    const auto ego_velocity = data_source_->GetVehicleDynamics().velocity;

    const auto obj_position = object_fusion.frenet_coords;
    const auto obj_lane_id = data_source_->GetGlobalLaneId(object_fusion.frenet_coords);
    const auto obj_velocity = object_fusion.velocity;

    const auto distance = units::length::meter_t{obj_position.s - ego_position.s};
    const auto is_near = units::math::abs(distance) < gkFarDistanceThreshold;
    const auto is_in_front = (distance > units::length::meter_t{0.0});
    const auto is_in_lane = (obj_lane_id == ego_lane_id);
    const auto is_higher_ego_velocity = (ego_velocity >= obj_velocity);

    return (is_near && is_in_front && is_in_lane && is_higher_ego_velocity);
}
units::velocity::meters_per_second_t VelocityPlanner::GetDeltaVelocity() const
{
    auto delta_velocity = units::velocity::meters_per_second_t{0.0};
    const auto sensor_fusion = data_source_->GetSensorFusion();

    const auto is_cipv_in_front = std::any_of(sensor_fusion.objs.begin(), sensor_fusion.objs.end(),
                                              [&](const auto& obj) { return IsClosestInPathVehicleInFront(obj); });
    if (is_cipv_in_front)
    {
        delta_velocity = (deceleration_ / frequency_);
    }
    else
    {
        delta_velocity = (acceleration_ / frequency_);
    }
    return delta_velocity;
}

void VelocityPlanner::CalculateTargetVelocity()
{
    const auto sensor_fusion = data_source_->GetSensorFusion();
    const auto speed_limit = data_source_->GetSpeedLimit();

    const auto delta_velocity = GetDeltaVelocity();
    target_velocity_ += delta_velocity;
    target_velocity_ = units::math::abs(target_velocity_);

    // apply speed limit rules
    target_velocity_ = units::math::min(target_velocity_, speed_limit);

    // keep at least 1 m/s velocity to keep engine running
    const auto min_velocity = units::velocity::meters_per_second_t{1.0};
    target_velocity_ = units::math::max(target_velocity_, min_velocity);

    std::stringstream log_stream;
    log_stream << "Calculated target velocity: " << target_velocity_ << std::endl;
    LOG_DEBUG("VelocityPlanner", log_stream.str());
}

units::velocity::meters_per_second_t VelocityPlanner::GetTargetVelocity() const { return target_velocity_; }

}  // namespace motion_planning