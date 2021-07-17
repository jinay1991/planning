///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#ifndef PLANNING_COMMUNICATION_TOPICS_H
#define PLANNING_COMMUNICATION_TOPICS_H

#include "middleware/communication/topic.h"
#include "planning/datatypes/lane.h"
#include "planning/datatypes/path_planning.h"
#include "planning/datatypes/sensor_fusion.h"
#include "planning/datatypes/trajectory.h"
#include "planning/datatypes/vehicle_dynamics.h"

#include <units.h>

namespace planning
{

class SensorFusionTopic : public middleware::Topic<SensorFusion>
{
};

class SelectedTrajectoryTopic : public middleware::Topic<Trajectory>
{
};

class SpeedLimitTopic : public middleware::Topic<units::velocity::meters_per_second_t>
{
};

class PreviousPathTopic : public middleware::Topic<PreviousPathGlobal>
{
};

class PreviousPathEndTopic : public middleware::Topic<FrenetCoordinates>
{
};

class VehicleDynamicsTopic : public middleware::Topic<VehicleDynamics>
{
};

class MapCoordinatesTopic : public middleware::Topic<MapCoordinatesList>
{
};
}  // namespace planning

#endif  /// PLANNING_COMMUNICATION_TOPICS_H
