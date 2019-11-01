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

bool RoadModelDataSource::IsLeftLane(const FrenetCoordinates& coords) const { return (coords.d > 0 && coords.d < 4); }
bool RoadModelDataSource::IsCenterLane(const FrenetCoordinates& coords) const { return (coords.d > 4 && coords.d < 8); }
bool RoadModelDataSource::IsRightLane(const FrenetCoordinates& coords) const { return (coords.d > 8 && coords.d < 12); }

GlobalLaneId RoadModelDataSource::GetGlobalLaneId(const FrenetCoordinates& coords) const
{
    if (IsLeftLane(coords))
    {
        return GlobalLaneId::kLeft;
    }
    else if (IsCenterLane(coords))
    {
        return GlobalLaneId::kCenter;
    }
    else if (IsRightLane(coords))
    {
        return GlobalLaneId::kRight;
    }
    else
    {
        return GlobalLaneId::kInvalid;
    }
}

GlobalLaneId RoadModelDataSource::GetGlobalLaneId() const { return GetGlobalLaneId(vehicle_dynamics_.frenet_coords); }
FrenetCoordinates RoadModelDataSource::GetPreviousPathEnd() const { return previous_path_end_frenet_; }
VehicleDynamics RoadModelDataSource::GetVehicleDynamics() const { return vehicle_dynamics_; }
MapCoordinatesList RoadModelDataSource::GetMapCoordinates() const { return map_coordinates_; }
PreviousPathGlobal RoadModelDataSource::GetPreviousPathInGlobalCoords() const { return previous_path_global_; }
SensorFusion RoadModelDataSource::GetSensorFusion() const { return sensor_fusion_; }

}  // namespace motion_planning
