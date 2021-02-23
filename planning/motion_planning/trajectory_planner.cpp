///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/trajectory_planner.h"

#include "planning/common/logging.h"

namespace planning
{
TrajectoryPlanner::TrajectoryPlanner(const DataSource& data_source) : data_source_{data_source} {}

Trajectories TrajectoryPlanner::GetPlannedTrajectories(const std::vector<Maneuver>& maneuvers) const
{
    const auto trajectories = GetTrajectories(maneuvers);
    return trajectories;
}

Trajectory TrajectoryPlanner::GetInitialTrajectory() const
{
    Trajectory trajectory{};

    const auto vehicle_dynamics = data_source_.GetVehicleDynamics();
    const auto previous_path_global = data_source_.GetPreviousPathInGlobalCoords();
    const auto previous_path_size = data_source_.GetPreviousPathInGlobalCoords().size();
    // no previous waypoints, initialize current waypoints
    if (previous_path_size < 2)
    {
        const GlobalCoordinates prev_position{vehicle_dynamics.global_coords.x - cos(vehicle_dynamics.yaw.value()),
                                              vehicle_dynamics.global_coords.y - sin(vehicle_dynamics.yaw.value())};

        trajectory.waypoints.push_back(prev_position);
        trajectory.waypoints.push_back(vehicle_dynamics.global_coords);

        trajectory.position = vehicle_dynamics.global_coords;
        trajectory.yaw = vehicle_dynamics.yaw;
    }
    else  // calculate new waypoints based on previous waypoints
    {
        const GlobalCoordinates prev_position[2] = {previous_path_global[previous_path_size - 2],
                                                    previous_path_global[previous_path_size - 1]};

        trajectory.waypoints.push_back(prev_position[0]);
        trajectory.waypoints.push_back(prev_position[1]);

        trajectory.position = prev_position[1];
        trajectory.yaw = units::angle::radian_t{
            atan2(prev_position[1].y - prev_position[0].y, prev_position[1].x - prev_position[0].x)};
    }

    return trajectory;
}

Trajectory TrajectoryPlanner::GetCalculatedTrajectory(const LaneId& lane_id) const
{
    // Waypoints based on previous path
    auto trajectory = GetInitialTrajectory();
    const auto vehicle_dynamics = data_source_.GetVehicleDynamics();

    // Set further waypoints based on going further along highway in desired lane
    const auto lane = static_cast<std::int32_t>(lane_id);
    trajectory.waypoints.push_back(
        GetGlobalCoordinates(FrenetCoordinates{vehicle_dynamics.frenet_coords.s + 30.0, 2.0 + (4.0 * lane), 0.0, 0.0}));
    trajectory.waypoints.push_back(
        GetGlobalCoordinates(FrenetCoordinates{vehicle_dynamics.frenet_coords.s + 60.0, 2.0 + (4.0 * lane), 0.0, 0.0}));
    trajectory.waypoints.push_back(
        GetGlobalCoordinates(FrenetCoordinates{vehicle_dynamics.frenet_coords.s + 90.0, 2.0 + (4.0 * lane), 0.0, 0.0}));

    // Shift and rotate points to local coordinates
    const auto yaw = trajectory.yaw.value();
    const auto position = trajectory.position;
    const auto shift_rotate_waypoints = [&](const auto& wp) {
        const auto shift_position = GlobalCoordinates{wp.x - position.x, wp.y - position.y};
        return GlobalCoordinates{((shift_position.x * cos(-yaw)) - (shift_position.y * sin(-yaw))),
                                 ((shift_position.x * sin(-yaw)) + (shift_position.y * cos(-yaw)))};
    };

    std::transform(
        trajectory.waypoints.begin(), trajectory.waypoints.end(), trajectory.waypoints.begin(), shift_rotate_waypoints);

    return trajectory;
}

GlobalLaneId TrajectoryPlanner::GetGlobalLaneId(const LaneId& lane_id) const
{
    const auto ego_global_lane_id = data_source_.GetGlobalLaneId();
    switch (lane_id)
    {
        case LaneId::kEgo:
            return ego_global_lane_id;
        case LaneId::kLeft:
            return ego_global_lane_id - 1;
        case LaneId::kRight:
            return ego_global_lane_id + 1;
        case LaneId::kInvalid:
        default:
            return GlobalLaneId::kInvalid;
    }
}

Trajectories TrajectoryPlanner::GetTrajectories(const std::vector<Maneuver>& maneuvers) const
{
    Trajectories trajectories{};
    const auto previous_path_global = data_source_.GetPreviousPathInGlobalCoords();
    const auto vehicle_dynamics = data_source_.GetVehicleDynamics();
    std::int32_t unique_id = 0;
    for (const auto& maneuver : maneuvers)
    {
        Trajectory trajectory{};
        const auto lane_id = maneuver.GetLaneId();

        /// update waypoints with old path inputs
        trajectory.waypoints.insert(
            trajectory.waypoints.end(), previous_path_global.begin(), previous_path_global.end());
        trajectory.position = vehicle_dynamics.global_coords;
        trajectory.yaw = vehicle_dynamics.yaw;
        trajectory.velocity = maneuver.GetVelocity();
        trajectory.unique_id = ++unique_id;
        trajectory.lane_id = lane_id;
        trajectory.global_lane_id = GetGlobalLaneId(lane_id);

        /// calculate further waypoints for next path
        const auto calculated_trajectory = GetCalculatedTrajectory(lane_id);

        /// update waypoints
        trajectory.waypoints.insert(
            trajectory.waypoints.end(), calculated_trajectory.waypoints.begin(), calculated_trajectory.waypoints.end());

        /// append to trajectories
        trajectories.push_back(trajectory);
    }

    std::stringstream log_stream;
    log_stream << "Previous Path: " << previous_path_global.size() << std::endl;
    if (!previous_path_global.empty())
    {
        const auto n_samples =
            std::min(static_cast<std::size_t>(previous_path_global.size()), static_cast<std::size_t>(10));
        std::for_each(previous_path_global.begin(),
                      previous_path_global.begin() + n_samples,
                      [&log_stream](const auto& wp) { log_stream << "     => " << wp << std::endl; });
        log_stream << "     => ... (more " << previous_path_global.size() - n_samples << " waypoints)" << std::endl;
    }
    log_stream << std::endl;

    log_stream << "Planned trajectories: " << trajectories.size() << std::endl;
    std::for_each(trajectories.begin(), trajectories.end(), [&log_stream](const auto& trajectory) {
        log_stream << " (+) " << trajectory << std::endl;
        const auto n_samples =
            std::min(static_cast<std::size_t>(trajectory.waypoints.size()), static_cast<std::size_t>(10));
        std::for_each(trajectory.waypoints.begin(),
                      trajectory.waypoints.begin() + n_samples,
                      [&log_stream](const auto& wp) { log_stream << "     => " << wp << std::endl; });
        log_stream << "     => ... (more " << trajectory.waypoints.size() - n_samples << " waypoints)" << std::endl;
    });

    LOG(INFO) << log_stream.str();
    return trajectories;
}

GlobalCoordinates TrajectoryPlanner::GetGlobalCoordinates(const FrenetCoordinates& frenet_coords) const
{
    std::int32_t prev_wp = -1;
    const auto map_coordinates = data_source_.GetMapCoordinates();
    while (frenet_coords.s > map_coordinates[prev_wp + 1].frenet_coords.s &&
           (prev_wp < static_cast<std::int32_t>(map_coordinates.size() - 1)))
    {
        ++prev_wp;
    }

    std::int32_t wp2 = (prev_wp + 1) % map_coordinates.size();

    double heading = atan2((map_coordinates[wp2].global_coords.y - map_coordinates[prev_wp].global_coords.y),
                           (map_coordinates[wp2].global_coords.x - map_coordinates[prev_wp].global_coords.x));
    // the x,y,s along the segment
    double seg_s = (frenet_coords.s - map_coordinates[prev_wp].frenet_coords.s);

    double seg_x = map_coordinates[prev_wp].global_coords.x + seg_s * cos(heading);
    double seg_y = map_coordinates[prev_wp].global_coords.y + seg_s * sin(heading);

    double perp_heading = heading - units::constants::detail::PI_VAL / 2;

    double x = seg_x + frenet_coords.d * cos(perp_heading);
    double y = seg_y + frenet_coords.d * sin(perp_heading);

    return {x, y};
}

}  // namespace planning
