///
/// @file trajectory.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_DOMAIN_MODEL_TRAJECTORY_H_
#define MOTION_PLANNING_DOMAIN_MODEL_TRAJECTORY_H_

#include <vector>

#include "motion_planning/domain_model/vehicle_dynamics.h"
#include "motion_planning/maneuver.h"

namespace motion_planning
{
/// @brief Trajectory
struct Trajectory
{
    /// @brief Trajectory Id (unique)
    std::int32_t unique_id;

    /// @brief Trajectory Waypoints in Global Coordinates
    std::vector<GlobalCoordinates> waypoints;

    /// @brief Ego Vehicle Position in Global Coordinates
    GlobalCoordinates position;

    /// @brief Trajectory Global LaneId
    LaneInformation::GlobalLaneId global_lane_id;

    /// @brief Trajectory Local LaneId
    LaneInformation::LaneId lane_id;

    /// @brief Ego Vehicle yaw (in radians)
    units::angle::radian_t yaw;

    /// @brief Trajectory Cost
    double cost{0.0};

    /// @brief Maneuver associated with Trajectory.
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

/// @brief String Stream for Trajectory information (used for printing verbose information)
inline std::ostream& operator<<(std::ostream& out, const Trajectory& trajectory)
{
    return out << "Trajectory{id: " << trajectory.unique_id << ", wp: " << trajectory.waypoints.size()
               << ", lane: " << trajectory.lane_id << ", global_lane: " << trajectory.global_lane_id
               << ", velocity: " << trajectory.maneuver.GetVelocity() << ", cost: " << trajectory.cost
               << ", yaw: " << trajectory.yaw << "}";
}

}  // namespace motion_planning

#endif  /// MOTION_PLANNING_DOMAIN_MODEL_TRAJECTORY_H_
