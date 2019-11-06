///
/// @file
///
#include <logging/logging.h>

#include "motion_planning/velocity_planner/velocity_planner.h"

namespace motion_planning
{
VelocityPlanner::VelocityPlanner(std::shared_ptr<IDataSource> data_source)
    : frequency_{25.0},
      speed_limit_{22.12848},  // 49.5 MPH
      deceleration_{-5.0},
      acceleration_{5.0},
      target_velocity_{speed_limit_},
      data_source_{data_source}
{
}

bool VelocityPlanner::IsClosestInPathVehicleInFront(const ObjectFusion& object_fusion) const
{
    const auto ego_lane_id = data_source_->GetGlobalLaneId();
    const auto ego_position = data_source_->GetPreviousPathEnd();
    const auto obj_position = object_fusion.frenet_coords;
    const auto obj_lane_id = data_source_->GetGlobalLaneId(object_fusion.frenet_coords);
    const auto is_near = (std::fabs(obj_position.s - ego_position.s) < gkFarDistanceThreshold.value());

    return (is_near && (obj_lane_id == ego_lane_id));
}

void VelocityPlanner::CalculateTargetVelocity()
{
    const auto sensor_fusion = data_source_->GetSensorFusion();

    const auto cipv_in_front_iterator =
        std::find_if(sensor_fusion.objs.begin(), sensor_fusion.objs.end(),
                     [&](const auto& obj) { return IsClosestInPathVehicleInFront(obj); });
    if (cipv_in_front_iterator != sensor_fusion.objs.end())
    {
        target_velocity_ += (deceleration_ / frequency_);
    }
    else
    {
        target_velocity_ += (acceleration_ / frequency_);
    }
    // follow object
    const auto follow_lane_velocity = units::math::max(target_velocity_, cipv_in_front_iterator->velocity);

    // apply speed limit rules
    const auto limited_velocity = units::math::abs(units::math::min(target_velocity_, speed_limit_));

    // consolidated target velocity
    target_velocity_ = units::math::min(limited_velocity, follow_lane_velocity);

    // keep at least 1 meters_per_second_t velocity
    target_velocity_ = units::math::max(target_velocity_, units::velocity::meters_per_second_t{1.0});

    std::stringstream log_stream;
    log_stream << "Calculated target velocity: " << target_velocity_ << std::endl;
    LOG_DEBUG("VelocityPlanner", log_stream.str());
}

units::velocity::meters_per_second_t VelocityPlanner::GetTargetVelocity() const { return target_velocity_; }

}  // namespace motion_planning