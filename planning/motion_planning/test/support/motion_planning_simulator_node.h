///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_MOTION_PLANNING_TEST_SUPPORT_MOTION_PLANNING_SIMULATOR_NODE_H
#define PLANNING_MOTION_PLANNING_TEST_SUPPORT_MOTION_PLANNING_SIMULATOR_NODE_H

#include "middleware/lifecycle/node.h"
#include "planning/motion_planning/data_source.h"

#include <units.h>

namespace planning
{
class MotionPlanningSimulatorNode : public middleware::Node
{
  public:
    explicit MotionPlanningSimulatorNode(middleware::IPubSubFactory& factory);

    void Init() override;
    void ExecuteStep() override;
    void Shutdown() override;

    void SetSpeedLimit(const units::velocity::meters_per_second_t speed_limit);
    void BlockEgoLane();

  private:
    DataSource data_source_;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TEST_SUPPORT_MOTION_PLANNING_SIMULATOR_NODE_H