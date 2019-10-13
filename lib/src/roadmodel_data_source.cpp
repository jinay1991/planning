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
    if (coords.d > 0 && coords.d < 4)  // left lane (near to double solid lane marking)
    {
        return GlobalLaneId::kLeft;
    }
    else if (coords.d > 4 && coords.d < 8)  // center lane
    {
        return GlobalLaneId::kCenter;
    }
    else if (coords.d > 8 && coords.d < 12)  // right lane (near to the edge of the road)
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
