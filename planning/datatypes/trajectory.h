///
/// @file trajectory.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_DATATYPES_TRAJECTORY_H_
#define PLANNING_DATATYPES_TRAJECTORY_H_

#include "planning/datatypes/vehicle_dynamics.h"

#include <units.h>

#include <cstdint>
#include <vector>

namespace planning
{
/// @brief Trajectory
struct Trajectory
{
    /// @brief Trajectory Id (unique). (default to Invalid Id -1)
    std::int32_t unique_id{-1};

    /// @brief Trajectory Waypoints in Global Coordinates
    std::vector<GlobalCoordinates> waypoints;

    /// @brief Ego Vehicle Position in Global Coordinates
    GlobalCoordinates position{};

    /// @brief Trajectory Global LaneId
    LaneInformation::GlobalLaneId global_lane_id{LaneInformation::GlobalLaneId::kInvalid};

    /// @brief Trajectory Local LaneId
    LaneInformation::LaneId lane_id{LaneInformation::LaneId::kInvalid};

    /// @brief Ego Vehicle yaw (in radians)
    units::angle::radian_t yaw{0.0};

    /// @brief Trajectory Cost
    double cost{0.0};

    /// @brief Drivability of Trajectory
    bool drivable{false};

    /// @brief Velocity (mps) for Trajectory.
    units::velocity::meters_per_second_t velocity{0.0};
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
               << ", drivable: " << std::boolalpha << trajectory.drivable << ", velocity: " << trajectory.velocity
               << ", cost: " << trajectory.cost << ", yaw: " << trajectory.yaw << "}";
}

}  // namespace planning

#endif  /// PLANNING_DATATYPES_TRAJECTORY_H_
