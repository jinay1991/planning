///
/// @file trajectory_builder.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef TRAJECTORY_BUILDER_H_
#define TRAJECTORY_BUILDER_H_

#include <units.h>

#include "motion_planning/domain_model/trajectory.h"
#include "motion_planning/maneuver.h"

using namespace motion_planning;

namespace
{
class TrajectoryBuilder
{
  public:
    TrajectoryBuilder()
    {
        trajectory_.global_lane_id = LaneInformation::GlobalLaneId::kCenter;
        trajectory_.waypoints.resize(10);
    }

    TrajectoryBuilder& WithPosition(const GlobalCoordinates& position)
    {
        trajectory_.position = position;
        return *this;
    }

    TrajectoryBuilder& WithCost(const double& cost)
    {
        trajectory_.cost = cost;
        return *this;
    }

    TrajectoryBuilder& WithTargetVelocity(const units::velocity::meters_per_second_t& target_velocity)
    {
        target_velocity_ = target_velocity;
        return *this;
    }

    TrajectoryBuilder& WithLaneId(const LaneInformation::LaneId& lane_id)
    {
        trajectory_.maneuver = Maneuver{lane_id, target_velocity_};
        trajectory_.lane_id = lane_id;
        return *this;
    }

    TrajectoryBuilder& WithGlobalLaneId(const LaneInformation::GlobalLaneId& global_lane_id)
    {
        trajectory_.global_lane_id = global_lane_id;
        return *this;
    }

    TrajectoryBuilder& WithYaw(const units::angle::radian_t& yaw)
    {
        trajectory_.yaw = yaw;
        return *this;
    }

    TrajectoryBuilder& WithWaypoints(const std::vector<GlobalCoordinates>& waypoints)
    {
        trajectory_.waypoints = waypoints;
        return *this;
    }

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

    Trajectory Build() const { return trajectory_; }

  private:
    Trajectory trajectory_;

    units::velocity::meters_per_second_t target_velocity_{10.0};
};
}  // namespace

#endif  /// TRAJECTORY_BUILDER_H_
