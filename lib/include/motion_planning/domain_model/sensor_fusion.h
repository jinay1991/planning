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
/// @brief Safe Distance value (in meters)
static const auto gkFarDistanceThreshold = units::length::meter_t{30.0};

/// @brief ObjectFusion
struct ObjectFusion
{
    /// @brief Object Id
    std::int32_t idx;

    /// @brief Object Position in Global Coordinates
    GlobalCoordinates global_coords;

    /// @brief Object Position in Frenet Coordinates
    FrenetCoordinates frenet_coords;

    /// @brief Object Velocity (current)
    units::velocity::meters_per_second_t velocity;
};

/// @brief SensorFusion
struct SensorFusion
{
    /// @brief List of Objects
    std::vector<ObjectFusion> objs;
};

}  // namespace motion_planning

#endif  /// MOTION_PLANNING_DOMAIN_MODEL_SENSOR_FUSION_H_
