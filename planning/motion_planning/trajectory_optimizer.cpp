///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/trajectory_optimizer.h"

#include "planning/common/logging.h"

#include <algorithm>
#include <sstream>
#include <vector>

namespace planning
{
TrajectoryOptimizer::TrajectoryOptimizer(const IDataSource& data_source) : data_source_{data_source} {}

Trajectories TrajectoryOptimizer::GetOptimizedTrajectories(const Trajectories& planned_trajectories) const
{
    auto optimized_trajectories = Trajectories{};

    std::transform(planned_trajectories.begin(),
                   planned_trajectories.end(),
                   std::back_inserter(optimized_trajectories),
                   [this](const auto& trajectory) { return GetOptimizedTrajectory(trajectory); });

    std::stringstream log_stream;
    log_stream << "Optimized trajectories: " << optimized_trajectories.size() << std::endl;
    std::for_each(optimized_trajectories.begin(), optimized_trajectories.end(), [&log_stream](const auto& trajectory) {
        log_stream << " (+) " << trajectory << std::endl;
        const auto n_samples =
            std::min(static_cast<std::size_t>(trajectory.waypoints.size()), static_cast<std::size_t>(10));
        std::for_each(trajectory.waypoints.begin(),
                      trajectory.waypoints.begin() + n_samples,
                      [&log_stream](const auto& wp) { log_stream << "     => " << wp << std::endl; });
        log_stream << "     => ... (more " << trajectory.waypoints.size() - n_samples << " waypoints)" << std::endl;
    });
    LOG(INFO) << log_stream.str();
    return optimized_trajectories;
}

Trajectory TrajectoryOptimizer::GetOptimizedTrajectory(const Trajectory& planned_trajectory) const
{
    auto optimized_trajectory = planned_trajectory;
    const auto previous_path_global = data_source_.GetPreviousPathInGlobalCoords();

    // keep only calculated waypoints from copied version of planned trajectory
    // erase preserve previous path waypoints
    optimized_trajectory.waypoints.erase(optimized_trajectory.waypoints.begin(),
                                         optimized_trajectory.waypoints.begin() + previous_path_global.size());

    // split waypoints to points_x and points_y for spline utility
    std::vector<double> points_x;
    std::vector<double> points_y;
    for (const auto& waypoint : optimized_trajectory.waypoints)
    {
        points_x.push_back(waypoint.x);
        points_y.push_back(waypoint.y);
    }

    tk::spline spline;

    spline.set_points(points_x, points_y);

    // spline waypoints at 30m intervals
    const auto target_position = GlobalCoordinates{30.0, spline(30.0)};
    const auto target_distance =
        sqrt((target_position.x * target_position.x) + (target_position.y * target_position.y));
    double x_add_on = 0.0;

    const auto yaw = optimized_trajectory.yaw;
    const auto position = optimized_trajectory.position;
    const auto target_velocity = optimized_trajectory.velocity.value();
    constexpr auto kTotalWaypoints = 50U;
    for (std::size_t i = 1; i <= kTotalWaypoints - previous_path_global.size(); i++)
    {
        const double N = (target_distance / (0.02 * target_velocity));
        double x_point = x_add_on + (target_position.x / N);
        double y_point = spline(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * units::math::cos(yaw) - y_ref * units::math::sin(yaw));
        y_point = (x_ref * units::math::sin(yaw) + y_ref * units::math::cos(yaw));

        x_point += position.x;
        y_point += position.y;

        optimized_trajectory.waypoints.push_back(GlobalCoordinates{x_point, y_point});
    }

    return optimized_trajectory;
}

}  // namespace planning
