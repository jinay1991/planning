///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_DATATYPES_SENSOR_FUSION_H
#define PLANNING_DATATYPES_SENSOR_FUSION_H

#include "planning/datatypes/vehicle_dynamics.h"

#include <vector>

namespace planning
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

}  // namespace planning

#endif  /// PLANNING_DATATYPES_SENSOR_FUSION_H
