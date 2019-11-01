///
/// @file
///
#include <sstream>

#include "logging/logging.h"
#include "motion_planning/trajectory_evaluator.h"

namespace motion_planning
{
TrajectoryEvaluator::TrajectoryEvaluator(std::shared_ptr<IDataSource>& data_source)
    : lane_evaluator_{std::make_unique<LaneEvaluator>(data_source)}
{
}

RatedTrajectories TrajectoryEvaluator::GetRatedTrajectories(const PlannedTrajectories& planned_trajectories) const
{
    RatedTrajectories rated_trajectories{};

    std::copy_if(planned_trajectories.begin(), planned_trajectories.end(), std::back_inserter(rated_trajectories),
                 [&](const auto& trajectory) { return lane_evaluator_->IsValidLane(trajectory.lane_id); });

    const auto adjust_costs = [&](const auto& trajectory) {
        auto rated_trajectory = trajectory;
        if (trajectory.lane_id != LaneInformation::LaneId::kEgo)
        {
            rated_trajectory.cost += 1;
        }
        return rated_trajectory;
    };
    std::transform(rated_trajectories.begin(), rated_trajectories.end(), rated_trajectories.begin(), adjust_costs);

    std::stringstream log_stream;
    log_stream << "Evaluated trajectories: " << rated_trajectories.size() << std::endl;
    std::for_each(rated_trajectories.begin(), rated_trajectories.end(),
                  [&log_stream](const auto& trajectory) { log_stream << " (+) " << trajectory << std::endl; });
    LOG_DEBUG("TrajectoryEvaluator", log_stream.str());
    return rated_trajectories;
}

}  // namespace motion_planning
