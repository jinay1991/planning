///
/// @file
///

#ifndef MOTION_PLANNING_I_DATA_SOURCE_H_
#define MOTION_PLANNING_I_DATA_SOURCE_H_

#include <motion_planning/domain_model/lane.h>
#include <motion_planning/domain_model/sensor_fusion.h>
#include <motion_planning/domain_model/trajectory.h>
#include <motion_planning/domain_model/vehicle_dynamics.h>

namespace motion_planning
{
class IDataSource
{
  public:
    // virtual void SetGlobalLaneId(const LaneInformation::GlobalLaneId& global_lane_id) = 0;
    virtual void SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics) = 0;
    virtual void SetMapCoordinates(const std::vector<MapCoordinates>& map_coordinates) = 0;
    virtual void SetPreviousPath(const std::vector<GlobalCoordinates>& previous_path_global) = 0;
    virtual void SetPreviousPathEnd(const FrenetCoordinates& frenet_coords) = 0;
    virtual void SetSensorFusion(const SensorFusion& sensor_fusion) = 0;

    virtual LaneInformation::GlobalLaneId GetGlobalLaneId() const = 0;
    virtual FrenetCoordinates GetPreviousPathEnd() const = 0;
    virtual VehicleDynamics GetVehicleDynamics() const = 0;
    virtual std::vector<MapCoordinates> GetMapCoordinates() const = 0;
    virtual std::vector<GlobalCoordinates> GetPreviousPathInGlobalCoords() const = 0;
    virtual SensorFusion GetSensorFusion() const = 0;
};
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_I_DATA_SOURCE_H_
