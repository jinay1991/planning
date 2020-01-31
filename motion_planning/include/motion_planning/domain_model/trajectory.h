///
/// @file
///
#ifndef MOTION_PLANNING_DOMAIN_MODEL_TRAJECTORY_H_
#define MOTION_PLANNING_DOMAIN_MODEL_TRAJECTORY_H_

#include <vector>

#include "motion_planning/domain_model/vehicle_dynamics.h"
#include "motion_planning/maneuver.h"

namespace motion_planning
{
struct Trajectory
{
    std::vector<GlobalCoordinates> waypoints;

    GlobalCoordinates position;

    LaneInformation::GlobalLaneId global_lane_id;
    LaneInformation::LaneId lane_id;

    units::angle::radian_t yaw;

    double cost{0.0};

    Maneuver maneuver{};
};

using Trajectories = std::vector<Trajectory>;

/// @brief Compare Trajectory based on Cost and Lane Assignments
inline bool operator>(const Trajectory& lhs, const Trajectory& rhs)
{
    using LaneId = LaneInformation::LaneId;

    return lhs.cost > rhs.cost ||
           (lhs.cost == rhs.cost && (lhs.lane_id != LaneId::kInvalid || rhs.lane_id != LaneId::kInvalid) &&
            lhs.lane_id < rhs.lane_id);
}

inline std::ostream& operator<<(std::ostream& out, const Trajectory& trajectory)
{
    return out << "Trajectory{wp: " << trajectory.waypoints.size() << ", lane: " << trajectory.lane_id
               << ", global_lane: " << trajectory.global_lane_id << ", velocity: " << trajectory.maneuver.GetVelocity()
               << ", cost: " << trajectory.cost << "}";
}

}  // namespace motion_planning

#endif  /// MOTION_PLANNING_DOMAIN_MODEL_TRAJECTORY_H_
