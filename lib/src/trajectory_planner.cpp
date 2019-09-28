///
/// @file
///
#include <motion_planning/trajectory_planner.h>
#include <sstream>

using namespace units::literals;

namespace motion_planning
{
TrajectoryPlanner::TrajectoryPlanner(std::shared_ptr<IDataSource>& data_source) : data_source_(data_source) {}

PlannedTrajectories TrajectoryPlanner::GetPlannedTrajectories(const std::vector<Maneuver>& maneuvers) const
{
    const auto trajectories = GetTrajectories(maneuvers);

    return trajectories;
}

Trajectory TrajectoryPlanner::GetInitialTrajectory() const
{
    Trajectory trajectory;

    const auto vehicle_dynamics = data_source_->GetVehicleDynamics();
    const auto previous_path_global = data_source_->GetPreviousPathInGlobalCoords();
    const auto previous_path_size = data_source_->GetPreviousPathInGlobalCoords().size();
    // no previous waypoints, initialize current waypoints
    if (previous_path_size < 2)
    {
        GlobalCoordinates prev_position{vehicle_dynamics.global_coords.x - cos(vehicle_dynamics.yaw.value()),
                                        vehicle_dynamics.global_coords.y - sin(vehicle_dynamics.yaw.value())};

        trajectory.waypoints.push_back(prev_position);
        trajectory.waypoints.push_back(vehicle_dynamics.global_coords);

        trajectory.position = vehicle_dynamics.global_coords;
        trajectory.yaw = vehicle_dynamics.yaw;
    }
    else  // calculate new waypoints based on previous waypoints
    {
        GlobalCoordinates prev_position[2] = {previous_path_global[previous_path_size - 2],
                                              previous_path_global[previous_path_size - 1]};

        trajectory.waypoints.push_back(prev_position[0]);
        trajectory.waypoints.push_back(prev_position[1]);

        trajectory.position = prev_position[1];
        trajectory.yaw = units::angle::radian_t{
            atan2(prev_position[1].y - prev_position[0].y, prev_position[1].x - prev_position[0].x)};
    }

    return trajectory;
}

Trajectory TrajectoryPlanner::GetOptimizedTrajectory(const Trajectory& calculated_trajectory,
                                                     const units::velocity::meters_per_second_t& target_velocity) const
{
    Trajectory optimized_trajectory;
    // split waypoints to ptsx and ptsy for spline utility
    std::vector<double> ptsx;
    std::vector<double> ptsy;
    std::transform(calculated_trajectory.waypoints.begin(), calculated_trajectory.waypoints.end(),
                   std::back_inserter(ptsx), [](const auto& wp) { return wp.x; });
    std::transform(calculated_trajectory.waypoints.begin(), calculated_trajectory.waypoints.end(),
                   std::back_inserter(ptsy), [](const auto& wp) { return wp.y; });

    tk::spline spline;

    spline.set_points(ptsx, ptsy);

    // spline waypoints at 30m intervals
    auto target_position = GlobalCoordinates{30.0, spline(30.0)};
    double target_dist = sqrt((target_position.x * target_position.x) + (target_position.y * target_position.y));

    double x_add_on = 0;

    const auto yaw = calculated_trajectory.yaw.value();
    const auto position = calculated_trajectory.position;

    for (std::size_t i = 1; i <= 50 - data_source_->GetPreviousPathInGlobalCoords().size(); i++)
    {
        double N = (target_dist / (0.02f * target_velocity.value()));
        double x_point = x_add_on + (target_position.x / N);
        double y_point = spline(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(yaw) - y_ref * sin(yaw));
        y_point = (x_ref * sin(yaw) + y_ref * cos(yaw));

        x_point += position.x;
        y_point += position.y;

        optimized_trajectory.waypoints.push_back(GlobalCoordinates{x_point, y_point});
    }

    return optimized_trajectory;
}

Trajectory TrajectoryPlanner::GetCalculatedTrajectory(const Maneuver::LaneId& lane_id) const
{
    // Waypoints based on previous path
    auto trajectory = GetInitialTrajectory();
    const auto vehicle_dynamics = data_source_->GetVehicleDynamics();
    // Set further waypoints based on going further along highway in desired lane
    std::int32_t lane = static_cast<std::int32_t>(lane_id);
    trajectory.waypoints.push_back(
        GetGlobalCoordinates(FrenetCoordinates{vehicle_dynamics.frenet_coords.s + 30, 2.0 + (4.0 * lane)}));
    trajectory.waypoints.push_back(
        GetGlobalCoordinates(FrenetCoordinates{vehicle_dynamics.frenet_coords.s + 60, 2.0 + (4.0 * lane)}));
    trajectory.waypoints.push_back(
        GetGlobalCoordinates(FrenetCoordinates{vehicle_dynamics.frenet_coords.s + 90, 2.0 + (4.0 * lane)}));

    // Shift and rotate points to local coordinates
    const auto yaw = trajectory.yaw.value();
    const auto position = trajectory.position;
    std::transform(trajectory.waypoints.begin(), trajectory.waypoints.end(), trajectory.waypoints.begin(),
                   [&yaw, &position](const auto& wp) -> GlobalCoordinates {
                       const GlobalCoordinates shift_position{wp.x - position.x, wp.y - position.y};
                       return GlobalCoordinates{((shift_position.x * cos(-yaw)) - (shift_position.y * sin(-yaw))),
                                                ((shift_position.x * sin(-yaw)) + (shift_position.y * cos(-yaw)))};
                   });

    return trajectory;
}
LaneInformation::GlobalLaneId TrajectoryPlanner::GetGlobalLaneId(const LaneInformation::LaneId& lane_id) const
{
    const auto ego_global_lane_id = data_source_->GetGlobalLaneId();
    switch (lane_id)
    {
        case LaneInformation::LaneId::kEgo:
            return ego_global_lane_id;
        case LaneInformation::LaneId::kLeft:
            return ego_global_lane_id - 1;
        case LaneInformation::LaneId::kRight:
            return ego_global_lane_id + 1;
        case LaneInformation::LaneId::kInvalid:
        default:
            return LaneInformation::GlobalLaneId::kInvalid;
    }
}
Trajectories TrajectoryPlanner::GetTrajectories(const std::vector<Maneuver>& maneuvers) const
{
    Trajectories trajectories;
    const auto previous_path_global = data_source_->GetPreviousPathInGlobalCoords();
    const auto vehicle_dynamics = data_source_->GetVehicleDynamics();
    for (const auto& maneuver : maneuvers)
    {
        Trajectory trajectory{};
        const auto lane_id = maneuver.GetLaneId();
        const auto target_velocity = maneuver.GetVelocity();

        /// update waypoints with old path inputs
        trajectory.waypoints.insert(trajectory.waypoints.end(), previous_path_global.begin(),
                                    previous_path_global.end());
        trajectory.position = vehicle_dynamics.global_coords;
        trajectory.yaw = vehicle_dynamics.yaw;
        trajectory.maneuver = maneuver;
        trajectory.lane_id = lane_id;
        trajectory.global_lane_id = GetGlobalLaneId(lane_id);

        /// calculate further waypoints for next path
        const auto calculated_trajectory = GetCalculatedTrajectory(lane_id);

        /// optimize calculated trajectory with spline equation to produce minimal jerk trajectory
        const auto optimized_trajectory = GetOptimizedTrajectory(calculated_trajectory, target_velocity);

        /// update waypoints
        trajectory.waypoints.insert(trajectory.waypoints.end(), optimized_trajectory.waypoints.begin(),
                                    optimized_trajectory.waypoints.end());

        /// append to trajectories
        trajectories.push_back(trajectory);
    }
    std::stringstream log_stream;
    log_stream << "Planned trajectories: " << trajectories.size() << std::endl;
    LOG_DEBUG("TrajectoryPlanner", log_stream.str());
    return trajectories;
}

GlobalCoordinates TrajectoryPlanner::GetGlobalCoordinates(const FrenetCoordinates& frenet_coords) const
{
    int prev_wp = -1;
    const auto map_coordinates = data_source_->GetMapCoordinates();
    while (frenet_coords.s > map_coordinates[prev_wp + 1].frenet_coords.s &&
           (prev_wp < (int)(map_coordinates.size() - 1)))
    {
        ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % map_coordinates.size();

    double heading = atan2((map_coordinates[wp2].global_coords.y - map_coordinates[prev_wp].global_coords.y),
                           (map_coordinates[wp2].global_coords.x - map_coordinates[prev_wp].global_coords.x));
    // the x,y,s along the segment
    double seg_s = (frenet_coords.s - map_coordinates[prev_wp].frenet_coords.s);

    double seg_x = map_coordinates[prev_wp].global_coords.x + seg_s * cos(heading);
    double seg_y = map_coordinates[prev_wp].global_coords.y + seg_s * sin(heading);

    double perp_heading = heading - PI() / 2;

    double x = seg_x + frenet_coords.d * cos(perp_heading);
    double y = seg_y + frenet_coords.d * sin(perp_heading);

    return {x, y};
}
}  // namespace motion_planning
