///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#include "planning/motion_planning/data_source.h"
#include "planning/motion_planning/motion_planning.h"

#include <sstream>

namespace planning
{
namespace
{

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* input, const std::size_t size)
{
    std::istringstream s{std::string{reinterpret_cast<const char*>(input), size}};
    DataSource data_source;

    VehicleDynamics vehicle_dynamics{};
    if (s.read(reinterpret_cast<char*>(&vehicle_dynamics), sizeof(vehicle_dynamics)))
    {
        data_source.SetVehicleDynamics(vehicle_dynamics);
    }

    MapCoordinatesList map_coordinates_list{};
    for (auto i = 0U; i < 10U; ++i)
    {
        MapCoordinates map_coordinate{};
        if (s.read(reinterpret_cast<char*>(&map_coordinate), sizeof(map_coordinate)))
        {
            map_coordinates_list.push_back(map_coordinate);
        }
    }
    data_source.SetMapCoordinates(map_coordinates_list);

    PreviousPathGlobal previous_path_global{};
    for (auto i = 0U; i < 10U; ++i)
    {
        GlobalCoordinates global_coordinate{};
        if (s.read(reinterpret_cast<char*>(&global_coordinate), sizeof(global_coordinate)))
        {
            previous_path_global.push_back(global_coordinate);
        }
    }
    data_source.SetPreviousPath(previous_path_global);

    FrenetCoordinates frenet_coordinates{};
    if (s.read(reinterpret_cast<char*>(&frenet_coordinates), sizeof(frenet_coordinates)))
    {
        data_source.SetPreviousPathEnd(frenet_coordinates);
    }

    SensorFusion sensor_fusion{};
    for (auto i = 0U; i < 10U; ++i)
    {
        ObjectFusion object_fusion{};
        if (s.read(reinterpret_cast<char*>(&object_fusion), sizeof(object_fusion)))
        {
            sensor_fusion.objs.push_back(object_fusion);
        }
    }
    data_source.SetSensorFusion(sensor_fusion);

    units::velocity::meters_per_second_t speed_limit{};
    if (s.read(reinterpret_cast<char*>(&speed_limit), sizeof(speed_limit)))
    {
        data_source.SetSpeedLimit(speed_limit);
    }

    MotionPlanning motion_planning{data_source};
    motion_planning.GenerateTrajectories();

    return 0;
}
}  // namespace
}  // namespace planning
