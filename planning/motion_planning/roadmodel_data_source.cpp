///
/// @file
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#include "planning/motion_planning/roadmodel_data_source.h"

namespace planning
{
RoadModelDataSource::RoadModelDataSource()
    : global_lane_id_{GlobalLaneId::kCenter},
      vehicle_dynamics_{},
      map_coordinates_{},
      previous_path_global_{},
      previous_path_end_frenet_{},
      sensor_fusion_{},
      speed_limit_{units::velocity::meters_per_second_t{22.12848}}
{
}

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

void RoadModelDataSource::SetPreviousPathEnd(const FrenetCoordinates& coords)
{
    previous_path_end_frenet_ = coords;
}

void RoadModelDataSource::SetSensorFusion(const SensorFusion& sensor_fusion)
{
    sensor_fusion_ = sensor_fusion;
}

void RoadModelDataSource::SetSpeedLimit(const units::velocity::meters_per_second_t& speed_limit)
{
    speed_limit_ = speed_limit;
}

GlobalLaneId RoadModelDataSource::GetGlobalLaneId(const FrenetCoordinates& coords) const
{
    GlobalLaneId global_lane_id{GlobalLaneId::kInvalid};
    if (IsLeftLane(coords))
    {
        global_lane_id = GlobalLaneId::kLeft;
    }
    else if (IsCenterLane(coords))
    {
        global_lane_id = GlobalLaneId::kCenter;
    }
    else if (IsRightLane(coords))
    {
        global_lane_id = GlobalLaneId::kRight;
    }
    else
    {
        global_lane_id = GlobalLaneId::kInvalid;
    }
    return global_lane_id;
}

GlobalLaneId RoadModelDataSource::GetGlobalLaneId() const
{
    return GetGlobalLaneId(vehicle_dynamics_.frenet_coords);
}

FrenetCoordinates RoadModelDataSource::GetPreviousPathEnd() const
{
    return previous_path_end_frenet_;
}

VehicleDynamics RoadModelDataSource::GetVehicleDynamics() const
{
    return vehicle_dynamics_;
}

MapCoordinatesList RoadModelDataSource::GetMapCoordinates() const
{
    return map_coordinates_;
}

PreviousPathGlobal RoadModelDataSource::GetPreviousPathInGlobalCoords() const
{
    return previous_path_global_;
}

SensorFusion RoadModelDataSource::GetSensorFusion() const
{
    return sensor_fusion_;
}

units::velocity::meters_per_second_t RoadModelDataSource::GetSpeedLimit() const
{
    return speed_limit_;
}

bool RoadModelDataSource::IsLeftLane(const FrenetCoordinates& coords)
{
    return (coords.d > 0.0 && coords.d < 4.0);
}

bool RoadModelDataSource::IsCenterLane(const FrenetCoordinates& coords)
{
    return (coords.d > 4.0 && coords.d < 8.0);
}

bool RoadModelDataSource::IsRightLane(const FrenetCoordinates& coords)
{
    return (coords.d > 8.0 && coords.d < 12.0);
}

}  // namespace planning
