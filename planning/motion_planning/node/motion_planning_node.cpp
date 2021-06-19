///
/// @file
/// @copyright Copyright (c) 2020-2021. MIT License.
///
#include "planning/motion_planning/node/motion_planning_node.h"

#include "middleware/lifecycle/node.h"
#include "planning/communication/topics.h"

namespace planning
{
MotionPlanningNode::MotionPlanningNode(middleware::IPubSubFactory& factory)
    : middleware::Node{"motion_planning_node", factory}, data_source_{}, motion_planning_{data_source_}
{
}

void MotionPlanningNode::Init()
{
    InitSubscriber();
    InitPublisher();
}

void MotionPlanningNode::ExecuteStep()
{
    motion_planning_.GenerateTrajectories();
}

void MotionPlanningNode::Shutdown() {}

void MotionPlanningNode::InitSubscriber()
{
    AddSubscriber<MapCoordinatesTopic>(
        [&data_source = data_source_](const auto& map_coordinates) { data_source.SetMapCoordinates(map_coordinates); });
    AddSubscriber<PreviousPathTopic>(
        [&data_source = data_source_](const auto& previous_path) { data_source.SetPreviousPath(previous_path); });
    AddSubscriber<PreviousPathEndTopic>([&data_source = data_source_](const auto& previous_path_end) {
        data_source.SetPreviousPathEnd(previous_path_end);
    });
    AddSubscriber<VehicleDynamicsTopic>([&data_source = data_source_](const auto& vehicle_dynamics) {
        data_source.SetVehicleDynamics(vehicle_dynamics);
    });
    AddSubscriber<SpeedLimitTopic>(
        [&data_source = data_source_](const auto& speed_limit) { data_source.SetSpeedLimit(speed_limit); });
    AddSubscriber<SensorFusionTopic>(
        [&data_source = data_source_](const auto& sensor_fusion) { data_source.SetSensorFusion(sensor_fusion); });
}

void MotionPlanningNode::InitPublisher()
{
    AddPublisher<SelectedTrajectoryTopic>(
        [&motion_planning = motion_planning_] { return motion_planning.GetSelectedTrajectory(); });
}

}  // namespace planning
