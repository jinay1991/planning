///
/// @file trajectory_optimizer.cpp
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include <algorithm>
#include <sstream>

#include "logging/logging.h"
#include "motion_planning/trajectory_optimizer.h"

namespace motion_planning
{
TrajectoryOptimizer::TrajectoryOptimizer(std::shared_ptr<IDataSource>& data_source) : data_source_{data_source} {}

Trajectories TrajectoryOptimizer::GetOptimizedTrajectories(const Trajectories& planned_trajectories) const
{
    auto optimized_trajectories = Trajectories{};

    std::transform(planned_trajectories.begin(), planned_trajectories.end(), std::back_inserter(optimized_trajectories),
                   [&](const auto& trajectory) { return GetOptimizedTrajectory(trajectory); });

    std::stringstream log_stream;
    log_stream << "Optimized trajectories: " << optimized_trajectories.size() << std::endl;
    std::for_each(optimized_trajectories.begin(), optimized_trajectories.end(), [&log_stream](const auto& trajectory) {
        log_stream << " (+) " << trajectory << std::endl;
        std::for_each(trajectory.waypoints.begin(), trajectory.waypoints.begin() + 10,
                      [&log_stream](const auto& wp) { log_stream << "     => " << wp << std::endl; });
        log_stream << "     => ... (more " << trajectory.waypoints.size() - 10 << " waypoints)" << std::endl;
    });
    LOG(DEBUG) << log_stream.str();
    return optimized_trajectories;
}

Trajectory TrajectoryOptimizer::GetOptimizedTrajectory(const Trajectory& planned_trajectory) const
{
    auto optimized_trajectory = planned_trajectory;
    const auto previous_path_global = data_source_->GetPreviousPathInGlobalCoords();

    // erase only calculated waypoints from copied version of planned trajectory
    // though preserve previous path waypoints
    optimized_trajectory.waypoints.erase(optimized_trajectory.waypoints.begin() + previous_path_global.size(),
                                         optimized_trajectory.waypoints.end());

    // split waypoints to points_x and points_y for spline utility
    std::vector<double> points_x;
    std::vector<double> points_y;
    std::transform(planned_trajectory.waypoints.begin() + previous_path_global.size(),
                   planned_trajectory.waypoints.end(), std::back_inserter(points_x),
                   [](const auto& wp) { return wp.x; });
    std::transform(planned_trajectory.waypoints.begin() + previous_path_global.size(),
                   planned_trajectory.waypoints.end(), std::back_inserter(points_y),
                   [](const auto& wp) { return wp.y; });

    tk::spline spline;

    spline.set_points(points_x, points_y);

    // spline waypoints at 30m intervals
    auto target_position = GlobalCoordinates{30.0, spline(30.0)};
    double target_dist = sqrt((target_position.x * target_position.x) + (target_position.y * target_position.y));
    double x_add_on = 0.0;

    const auto yaw = planned_trajectory.yaw;
    const auto position = planned_trajectory.position;
    const auto target_velocity = planned_trajectory.maneuver.GetVelocity();

    for (std::size_t i = 1; i <= 50 - previous_path_global.size(); i++)
    {
        double N = (target_dist / (0.02f * target_velocity.value()));
        double x_point = x_add_on + (target_position.x / N);
        double y_point = spline(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(yaw.value()) - y_ref * sin(yaw.value()));
        y_point = (x_ref * sin(yaw.value()) + y_ref * cos(yaw.value()));

        x_point += position.x;
        y_point += position.y;

        optimized_trajectory.waypoints.push_back(GlobalCoordinates{x_point, y_point});
    }

    return optimized_trajectory;
}

}  // namespace motion_planning