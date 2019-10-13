///
/// @file
///

#ifndef MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H_
#define MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H_

#include <motion_planning/i_data_source.h>

namespace motion_planning
{
class RoadModelDataSource : public IDataSource
{
  public:
    RoadModelDataSource();

    void SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics) override;
    void SetMapCoordinates(const MapCoordinatesList& map_coordinates) override;
    void SetPreviousPath(const PreviousPathGlobal& previous_path_global) override;
    void SetPreviousPathEnd(const FrenetCoordinates& frenet_coords) override;
    void SetSensorFusion(const SensorFusion& sensor_fusion) override;

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
