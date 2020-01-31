///
/// @file sensor_fusion.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///

#ifndef MOTION_PLANNING_DOMAIN_MODEL_SENSOR_FUSION_H_
#define MOTION_PLANNING_DOMAIN_MODEL_SENSOR_FUSION_H_

#include <vector>

#include "motion_planning/domain_model/vehicle_dynamics.h"

namespace motion_planning
{
static const auto gkFarDistanceThreshold = units::length::meter_t{30.0};

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
