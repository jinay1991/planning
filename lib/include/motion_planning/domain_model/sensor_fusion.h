///
/// @file
///

#ifndef MOTION_PLANNING_DOMAIN_MODEL_SENSOR_FUSION_H_
#define MOTION_PLANNING_DOMAIN_MODEL_SENSOR_FUSION_H_

#include <motion_planning/domain_model/vehicle_dynamics.h>
#include <vector>

namespace motion_planning
{
struct ObjectFusion
{
    std::int32_t idx;
    GlobalCoordinates global_coords;
    FrenetCoordinates frenet_coords;
    units::velocity::meters_per_second_t velocity;
};

struct SensorFusion
{
    std::vector<ObjectFusion> objs;
};

}  // namespace motion_planning

#endif  /// MOTION_PLANNING_DOMAIN_MODEL_SENSOR_FUSION_H_
