///
/// @file
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#include "planning/motion_planning/trajectory_evaluator.h"
#include "planning/common/logging/logging.h"

#include <sstream>

namespace planning
{
TrajectoryEvaluator::TrajectoryEvaluator(std::shared_ptr<IDataSource>& data_source)
    : lane_evaluator_{std::make_unique<LaneEvaluator>(data_source)}
{
}

TrajectoryEvaluator::~TrajectoryEvaluator() {}

Trajectories TrajectoryEvaluator::GetRatedTrajectories(const Trajectories& optimized_trajectories) const
{
    Trajectories rated_trajectories{};

    // discard invalid lane trajectories
    std::copy_if(optimized_trajectories.begin(), optimized_trajectories.end(), std::back_inserter(rated_trajectories),
                 [&](const auto& trajectory) {
                     return trajectory.global_lane_id != GlobalLaneId::kInvalid &&
                            lane_evaluator_->IsValidLane(trajectory.lane_id);
                 });

    /// @todo Improve Cost adjustment algorithm
    // update costs for each trajectory
    const auto adjust_costs = [&](const auto& trajectory) {
        auto rated_trajectory = trajectory;
        rated_trajectory.drivable = lane_evaluator_->IsDrivableLane(trajectory.lane_id);
        if (!rated_trajectory.drivable)
        {
            rated_trajectory.cost = std::numeric_limits<double>::infinity();
        }
        else if (trajectory.lane_id != LaneId::kEgo)
        {
            rated_trajectory.cost += 1;
        }
        else
        {
            // Ego Lane cost to minimal if drivable (i.e. cost=0)
        }

        return rated_trajectory;
    };
    std::transform(rated_trajectories.begin(), rated_trajectories.end(), rated_trajectories.begin(), adjust_costs);

    std::stringstream log_stream;
    log_stream << "Evaluated trajectories: " << rated_trajectories.size() << std::endl;
    std::for_each(rated_trajectories.begin(), rated_trajectories.end(),
                  [&log_stream](const auto& trajectory) { log_stream << " (+) " << trajectory << std::endl; });
    LOG(INFO) << log_stream.str();
    return rated_trajectories;
}

}  // namespace planning
