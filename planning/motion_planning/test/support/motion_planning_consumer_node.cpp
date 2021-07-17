///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/motion_planning/test/support/motion_planning_consumer_node.h"

#include "middleware/lifecycle/node.h"
#include "planning/communication/topics.h"

namespace planning
{

MotionPlanningConsumerNode::MotionPlanningConsumerNode(middleware::IPubSubFactory& factory)
    : middleware::Node{"consumer_node", factory}, selected_trajectory_message_{}
{
}

void MotionPlanningConsumerNode::Init()
{
    AddSubscriber<SelectedTrajectoryTopic>([&selected_trajectory_message = selected_trajectory_message_](
                                               const auto& data) { selected_trajectory_message = data; });
}

void MotionPlanningConsumerNode::ExecuteStep() {}

void MotionPlanningConsumerNode::Shutdown() {}

const Trajectory& MotionPlanningConsumerNode::GetSelectedTrajectory() const
{
    return selected_trajectory_message_;
}

}  // namespace planning
