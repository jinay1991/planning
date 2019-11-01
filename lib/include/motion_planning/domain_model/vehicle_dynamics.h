///
/// @file
///
#ifndef MOTION_PLANNING_DOMAIN_MODEL_VEHICLE_DYNAMICS_H_
#define MOTION_PLANNING_DOMAIN_MODEL_VEHICLE_DYNAMICS_H_

#include <units.h>
#include <cstdint>

#include "motion_planning/domain_model/lane.h"

namespace motion_planning
{
struct GlobalCoordinates
{
    double x;
    double y;
};

struct FrenetCoordinates
{
    double s;
    double d;

    /// Frenet d unit normal vector (split up into the x component, and the y component
    /// used to store map waypoints
    double dx;
    double dy;
};

struct MapCoordinates
{
    GlobalCoordinates global_coords;
    FrenetCoordinates frenet_coords;
};

struct VehicleDynamics
{
    LaneInformation::LaneId lane_id;
    LaneInformation::GlobalLaneId global_lane_id;

    units::velocity::meters_per_second_t velocity;

    GlobalCoordinates global_coords;
    FrenetCoordinates frenet_coords;

    units::angle::radian_t yaw;
};

}  // namespace motion_planning

#endif  /// MOTION_PLANNING_DOMAIN_MODEL_LANE_H_
