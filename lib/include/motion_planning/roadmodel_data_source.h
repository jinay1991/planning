///
/// @file
///

#ifndef MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H_
#define MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H_

#include <motion_planning/domain_model/lane.h>
#include <motion_planning/domain_model/sensor_fusion.h>
#include <motion_planning/domain_model/vehicle_dynamics.h>
#include <motion_planning/i_data_source.h>

namespace motion_planning
{
using PreviousPathGlobal = std::vector<GlobalCoordinates>;
using PreviousPathFrenet = std::vector<FrenetCoordinates>;
using MapCoordinatesList = std::vector<MapCoordinates>;
using GlobalLaneId = LaneInformation::GlobalLaneId;

class RoadModelDataSource : public IDataSource
{
  public:
    RoadModelDataSource();

    explicit RoadModelDataSource(const GlobalLaneId& lane_id, const VehicleDynamics& vehicle_dynamics,
                                 const MapCoordinatesList& map_coords, const PreviousPathGlobal& previous_path_global,
                                 const SensorFusion& sensor_fusion);

    virtual void SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics) override;
    virtual void SetMapCoordinates(const MapCoordinatesList& map_coordinates) override;
    virtual void SetPreviousPath(const PreviousPathGlobal& previous_path_global) override;
    virtual void SetPreviousPathEnd(const FrenetCoordinates& frenet_coords) override;
    virtual void SetSensorFusion(const SensorFusion& sensor_fusion) override;

    GlobalLaneId GetGlobalLaneId() const override;
    FrenetCoordinates GetPreviousPathEnd() const override;
    VehicleDynamics GetVehicleDynamics() const override;
    MapCoordinatesList GetMapCoordinates() const override;
    PreviousPathGlobal GetPreviousPathInGlobalCoords() const override;
    SensorFusion GetSensorFusion() const override;

  private:
    GlobalLaneId global_lane_id_{GlobalLaneId::kInvalid};
    VehicleDynamics vehicle_dynamics_{};
    MapCoordinatesList map_coordinates_{};
    PreviousPathGlobal previous_path_global_{};
    FrenetCoordinates previous_path_end_frenet_{};
    SensorFusion sensor_fusion_{};
};
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H_
