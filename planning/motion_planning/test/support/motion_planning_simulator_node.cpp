///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/motion_planning/test/support/motion_planning_simulator_node.h"

#include "middleware/lifecycle/node.h"
#include "planning/communication/topics.h"
#include "planning/datatypes/sensor_fusion.h"
#include "planning/datatypes/vehicle_dynamics.h"
#include "planning/motion_planning/test/support/builders/data_source_builder.h"
#include "planning/motion_planning/test/support/map_coordinates.h"

#include <limits>

namespace planning
{
using namespace units::literals;

MotionPlanningSimulatorNode::MotionPlanningSimulatorNode(middleware::IPubSubFactory& factory)
    : middleware::Node{"simulator_node", factory}, data_source_{}
{
    data_source_.SetMapCoordinates(kHighwayMap);
    data_source_.SetPreviousPath(PreviousPathGlobal{});
}

void MotionPlanningSimulatorNode::Init()
{
    AddPublisher<MapCoordinatesTopic>([&data_source = data_source_] { return data_source.GetMapCoordinates(); });
    AddPublisher<PreviousPathTopic>(
        [&data_source = data_source_] { return data_source.GetPreviousPathInGlobalCoords(); });
    AddPublisher<PreviousPathEndTopic>([&data_source = data_source_] { return data_source.GetPreviousPathEnd(); });
    AddPublisher<VehicleDynamicsTopic>([&data_source = data_source_] { return data_source.GetVehicleDynamics(); });
    AddPublisher<SpeedLimitTopic>([&data_source = data_source_] { return data_source.GetSpeedLimit(); });
    AddPublisher<SensorFusionTopic>([&data_source = data_source_] { return data_source.GetSensorFusion(); });
}

void MotionPlanningSimulatorNode::ExecuteStep() {}

void MotionPlanningSimulatorNode::Shutdown() {}

void MotionPlanningSimulatorNode::SetSpeedLimit(const units::velocity::meters_per_second_t speed_limit)
{
    data_source_.SetSpeedLimit(speed_limit);
}

void MotionPlanningSimulatorNode::BlockEgoLane()
{
    constexpr auto kObjectDistance = 10.0_m;
    constexpr auto kObjectVelocity = 10.0_kph;
    constexpr auto kEgoVelocity = 30.0_kph;

    data_source_ = DataSourceBuilder()
                       .WithPreviousPath(PreviousPathGlobal{})
                       .WithVelocity(kEgoVelocity)
                       .WithMapCoordinates(kHighwayMap)
                       .WithGlobalLaneId(GlobalLaneId::kCenter)
                       .WithDistance(kObjectDistance)
                       .WithObjectInLane(GlobalLaneId::kCenter, kObjectVelocity)
                       .Build();
}

}  // namespace planning
