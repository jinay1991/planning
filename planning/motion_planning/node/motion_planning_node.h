///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_MOTION_PLANNING_NODE_MOTION_PLANNING_NODE_H
#define PLANNING_MOTION_PLANNING_NODE_MOTION_PLANNING_NODE_H

#include "middleware/lifecycle/node.h"
#include "planning/motion_planning/data_source.h"
#include "planning/motion_planning/motion_planning.h"

namespace planning
{
class MotionPlanningNode : public middleware::Node
{
  public:
    explicit MotionPlanningNode(middleware::IPubSubFactory& factory);

    void Init() override;
    void ExecuteStep() override;
    void Shutdown() override;

  private:
    void InitSubscriber();
    void InitPublisher();

    DataSource data_source_;
    MotionPlanning motion_planning_;
};

}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_NODE_MOTION_PLANNING_NODE_H