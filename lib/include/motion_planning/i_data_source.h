///
/// @file
///

#ifndef MOTION_PLANNING_I_DATA_SOURCE_H_
#define MOTION_PLANNING_I_DATA_SOURCE_H_

#include "motion_planning/domain_model/lane.h"
#include "motion_planning/domain_model/sensor_fusion.h"
#include "motion_planning/domain_model/trajectory.h"
#include "motion_planning/domain_model/vehicle_dynamics.h"

namespace motion_planning
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using MapCoordinatesList = std::vector<MapCoordinates>;
using PreviousPathGlobal = std::vector<GlobalCoordinates>;
using PreviousPathFrenet = std::vector<FrenetCoordinates>;

class IDataSource
{
  public:
    virtual void SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics) = 0;
    virtual void SetMapCoordinates(const MapCoordinatesList& map_coordinates) = 0;
    virtual void SetPreviousPath(const PreviousPathGlobal& previous_path_global) = 0;
    virtual void SetPreviousPathEnd(const FrenetCoordinates& frenet_coords) = 0;
    virtual void SetSensorFusion(const SensorFusion& sensor_fusion) = 0;

    virtual GlobalLaneId GetGlobalLaneId(const FrenetCoordinates& frenet_coords) const = 0;
    virtual GlobalLaneId GetGlobalLaneId() const = 0;
    virtual FrenetCoordinates GetPreviousPathEnd() const = 0;
    virtual VehicleDynamics GetVehicleDynamics() const = 0;
    virtual MapCoordinatesList GetMapCoordinates() const = 0;
    virtual PreviousPathGlobal GetPreviousPathInGlobalCoords() const = 0;
    virtual SensorFusion GetSensorFusion() const = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_DATA_SOURCE_H_
