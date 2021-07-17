///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_MOTION_PLANNING_TEST_SUPPORT_MOTION_PLANNING_CONSUMER_NODE_H
#define PLANNING_MOTION_PLANNING_TEST_SUPPORT_MOTION_PLANNING_CONSUMER_NODE_H

#include "middleware/lifecycle/node.h"
#include "planning/datatypes/trajectory.h"

namespace planning
{
class MotionPlanningConsumerNode : public middleware::Node
{
  public:
    explicit MotionPlanningConsumerNode(middleware::IPubSubFactory& factory);

    void Init() override;
    void ExecuteStep() override;
    void Shutdown() override;

    const Trajectory& GetSelectedTrajectory() const;

  private:
    Trajectory selected_trajectory_message_;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TEST_SUPPORT_MOTION_PLANNING_CONSUMER_NODE_H