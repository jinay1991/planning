///
/// @file trajectory_builder.h
/// @brief Contains builder utility for Trajectory Object built-up.
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_TEST_TRAJECTORY_BUILDER_H_
#define PLANNING_MOTION_PLANNING_TEST_TRAJECTORY_BUILDER_H_

#include "planning/datatypes/trajectory.h"

#include <units.h>

namespace planning
{
namespace
{
/// @brief Trajectory Object Builder
class TrajectoryBuilder
{
  public:
    /// @brief Constructor.
    TrajectoryBuilder()
    {
        target_velocity_ = units::velocity::meters_per_second_t{10.0};
        trajectory_.global_lane_id = LaneInformation::GlobalLaneId::kCenter;
        trajectory_.waypoints.resize(10);
    }

    /// @brief Build Trajectory with Position (Global Coordinates)
    TrajectoryBuilder& WithPosition(const GlobalCoordinates& position)
    {
        trajectory_.position = position;
        return *this;
    }

    /// @brief Build Trajectory with Trajectory Cost
    TrajectoryBuilder& WithCost(const double& cost)
    {
        trajectory_.cost = cost;
        return *this;
    }

    /// @brief Build Trajectory with Trajectory Velocity
    TrajectoryBuilder& WithTargetVelocity(const units::velocity::meters_per_second_t& target_velocity)
    {
        trajectory_.velocity = target_velocity;
        return *this;
    }

    /// @brief Build Trajectory with Trajectory Lane Id
    TrajectoryBuilder& WithLaneId(const LaneInformation::LaneId& lane_id)
    {
        trajectory_.lane_id = lane_id;
        return *this;
    }

    /// @brief Build Trajectory with Trajectory Global Lane Id
    TrajectoryBuilder& WithGlobalLaneId(const LaneInformation::GlobalLaneId& global_lane_id)
    {
        trajectory_.global_lane_id = global_lane_id;
        return *this;
    }

    /// @brief Build Trajectory with Trajectory Yaw (radians)
    TrajectoryBuilder& WithYaw(const units::angle::radian_t& yaw)
    {
        trajectory_.yaw = yaw;
        return *this;
    }

    /// @brief Build Trajectory with Trajectory Waypoints (Global Coordinates)
    TrajectoryBuilder& WithWaypoints(const std::vector<GlobalCoordinates>& waypoints)
    {
        trajectory_.waypoints = waypoints;
        return *this;
    }

    /// @brief Build Trajectory with Trajectory Waypoints (Global Coordinates)
    TrajectoryBuilder& WithWaypoints(const GlobalCoordinates& start_position, const units::angle::radian_t start_yaw,
                                     const std::size_t count, const units::length::meter_t displacement)
    {
        trajectory_.waypoints.clear();
        for (auto idx = 0U; idx < count; ++idx)
        {
            trajectory_.waypoints.push_back(
                GlobalCoordinates{start_position.x + (idx * displacement.value() * cos(start_yaw.value())),
                                  start_position.y + (idx * displacement.value() * sin(start_yaw.value()))});
        }
        return *this;
    }

    /// @brief Build Trajectory Object.
    Trajectory Build() const { return trajectory_; }

  private:
    /// @brief Target Velocity for the Trajectory.
    units::velocity::meters_per_second_t target_velocity_;

    /// @brief Trajectory
    Trajectory trajectory_;
};
}  // namespace
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TEST_TRAJECTORY_BUILDER_H_
