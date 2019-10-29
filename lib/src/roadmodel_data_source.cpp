///
/// @file
///
#include <motion_planning/roadmodel_data_source.h>

namespace motion_planning
{
RoadModelDataSource::RoadModelDataSource() : global_lane_id_{GlobalLaneId::kCenter} {}

void RoadModelDataSource::SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics)
{
    vehicle_dynamics_ = vehicle_dynamics;
}
void RoadModelDataSource::SetMapCoordinates(const MapCoordinatesList& map_coordinates)
{
    map_coordinates_ = map_coordinates;
}
void RoadModelDataSource::SetPreviousPath(const PreviousPathGlobal& previous_path_global)
{
    previous_path_global_ = previous_path_global;
}
void RoadModelDataSource::SetPreviousPathEnd(const FrenetCoordinates& frenet_coords)
{
    previous_path_end_frenet_ = frenet_coords;
}

void RoadModelDataSource::SetSensorFusion(const SensorFusion& sensor_fusion) { sensor_fusion_ = sensor_fusion; }

GlobalLaneId RoadModelDataSource::GetGlobalLaneId() const
{
    const auto coords = vehicle_dynamics_.frenet_coords;
    const auto is_left_lane = (coords.d > 0 && coords.d < 4);
    const auto is_center_lane = (coords.d > 4 && coords.d < 8);
    const auto is_right_lane = (coords.d > 8 && coords.d < 12);

    if (is_left_lane)
    {
        return GlobalLaneId::kLeft;
    }
    else if (is_center_lane)
    {
        return GlobalLaneId::kCenter;
    }
    else if (is_right_lane)
    {
        return GlobalLaneId::kRight;
    }
    else
    {
        return GlobalLaneId::kInvalid;
    }
}

FrenetCoordinates RoadModelDataSource::GetPreviousPathEnd() const { return previous_path_end_frenet_; }
VehicleDynamics RoadModelDataSource::GetVehicleDynamics() const { return vehicle_dynamics_; }
MapCoordinatesList RoadModelDataSource::GetMapCoordinates() const { return map_coordinates_; }
PreviousPathGlobal RoadModelDataSource::GetPreviousPathInGlobalCoords() const { return previous_path_global_; }
SensorFusion RoadModelDataSource::GetSensorFusion() const { return sensor_fusion_; }

}  // namespace motion_planning
