///
/// @file
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#include "planning/motion_planning/trajectory_prioritizer.h"
#include "planning/common/logging.h"

namespace planning
{
namespace internal
{
template <typename T>
void PrintQueue(T q)
{
    std::stringstream log_stream;
    log_stream << "Prioritized trajectories: " << q.size() << std::endl;
    std::int32_t idx = 1;
    while (!q.empty())
    {
        log_stream << "  " << (idx++) << ". " << q.top() << std::endl;
        q.pop();
    }
    LOG(INFO) << log_stream.str();
}
}  // namespace internal

TrajectoryPrioritizer::~TrajectoryPrioritizer() {}

PrioritizedTrajectories TrajectoryPrioritizer::GetPrioritizedTrajectories(const Trajectories& trajectories) const
{
    PrioritizedTrajectories prioritized_trajectories;
    for (auto& trajectory : trajectories)
    {
        prioritized_trajectories.push(trajectory);
    }
    internal::PrintQueue(prioritized_trajectories);
    return prioritized_trajectories;
}

}  // namespace planning
