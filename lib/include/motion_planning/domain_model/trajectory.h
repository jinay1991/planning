///
/// @file
///
#ifndef MOTION_PLANNING_DOMAIN_MODEL_TRAJECTORY_H_
#define MOTION_PLANNING_DOMAIN_MODEL_TRAJECTORY_H_

#include <motion_planning/domain_model/vehicle_dynamics.h>
#include <motion_planning/maneuver.h>
#include <vector>

namespace motion_planning
{
struct Trajectory
{
    std::vector<GlobalCoordinates> waypoints;

    GlobalCoordinates position;

    units::angle::radian_t yaw;

    double cost{0};

    Maneuver maneuver{};
};
/// @brief Compare Trajectory based on Cost and Lane Assignments
inline bool operator>(const Trajectory& lhs, const Trajectory& rhs)
{
    return lhs.cost > rhs.cost || (lhs.cost == rhs.cost && lhs.maneuver.GetLaneId() < rhs.maneuver.GetLaneId());
}

inline std::ostream& operator<<(std::ostream& out, const Trajectory& trajectory)
{
    return out << "Trajectory{wp: " << trajectory.waypoints.size() << ", lane: " << trajectory.maneuver.GetLaneId()
               << ", velocity: " << trajectory.maneuver.GetVelocity() << ", cost: " << trajectory.cost << "}";
}
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_DOMAIN_MODEL_TRAJECTORY_H_
