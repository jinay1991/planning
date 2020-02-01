///
/// @file vehicle_dynamics.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_DOMAIN_MODEL_VEHICLE_DYNAMICS_H_
#define MOTION_PLANNING_DOMAIN_MODEL_VEHICLE_DYNAMICS_H_

#include <units.h>
#include <cstdint>

#include "motion_planning/domain_model/lane.h"

namespace motion_planning
{
/// @brief Global Coordinates
struct GlobalCoordinates
{
    /// @brief x axis value
    double x;

    /// @brief y axis value
    double y;
};

/// @brief Frenet Coordinates
struct FrenetCoordinates
{
    /// @brief Longitudinal Value
    double s;

    /// @brief Latitudinal Value
    double d;

    /// @brief Latitudinal unit normal vector (x component)
    /// @note used to store map waypoints
    double dx;

    /// @brief Latitudinal unit normal vector (y component)
    /// @note used to store map waypoints
    double dy;
};

/// @brief Map Points
struct MapCoordinates
{
    /// @brief Map Point (Global Coordinates)
    GlobalCoordinates global_coords;

    /// @brief Map Point (Frenet Coordinates)
    FrenetCoordinates frenet_coords;
};

/// @brief VehicleDynamics
struct VehicleDynamics
{
    /// @brief LaneId (Local Coordinates)
    LaneInformation::LaneId lane_id;

    /// @brief LaneId (Global Coordinates)
    LaneInformation::GlobalLaneId global_lane_id;

    /// @brief Current Velocity
    units::velocity::meters_per_second_t velocity;

    /// @brief Current Position (Global Coordinates)
    GlobalCoordinates global_coords;

    /// @brief Current Position (Frenet Coordinates)
    FrenetCoordinates frenet_coords;

    /// @brief Current Yaw for Ego Vehicle
    units::angle::radian_t yaw;
};

}  // namespace motion_planning

#endif  /// MOTION_PLANNING_DOMAIN_MODEL_LANE_H_
