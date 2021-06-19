///
/// @file
/// @brief Contains builder utility for Data Source object built-up.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_TEST_SUPPORT_BUILDERS_DATA_SOURCE_BUILDER_H
#define PLANNING_MOTION_PLANNING_TEST_SUPPORT_BUILDERS_DATA_SOURCE_BUILDER_H

#include "planning/datatypes/lane.h"
#include "planning/datatypes/sensor_fusion.h"
#include "planning/datatypes/vehicle_dynamics.h"
#include "planning/motion_planning/data_source.h"
#include "planning/motion_planning/test/support/builders/sensor_fusion_builder.h"
#include "planning/motion_planning/test/support/map_coordinates.h"

#include <units.h>

#include <memory>

namespace planning
{
namespace
{
/// @brief Data Source builder class
class DataSourceBuilder
{
  public:
    /// @brief Constructor.
    DataSourceBuilder() : data_source_{}
    {
        this->WithFakePreviousPath(50)
            .WithMapCoordinates(kHighwayMap)
            .WithDistance(units::length::meter_t{24.0})
            .WithGlobalLaneId(GlobalLaneId::kCenter)
            .WithVelocity(units::velocity::meters_per_second_t{17.0})
            .WithSpeedLimit(units::velocity::meters_per_second_t{22.12848});
    }

    /// @brief Build Data Source Object with Global Lane Id information
    DataSourceBuilder& WithGlobalLaneId(const GlobalLaneId global_lane_id)
    {
        const auto lateral_distance = units::length::meter_t{GetCoordinates(global_lane_id).d};
        previous_path_end_frenet_.d = lateral_distance.value();
        vehicle_dynamics_.frenet_coords.d = lateral_distance.value();
        return *this;
    }

    /// @brief Build Data Source Object with Ego Velocity
    DataSourceBuilder& WithVelocity(const units::velocity::meters_per_second_t velocity)
    {
        vehicle_dynamics_.velocity = velocity;
        return *this;
    }

    /// @brief Build Data Source Object with Ego Position (Frenet Coordinates)
    DataSourceBuilder& WithFrenetCoordinates(const FrenetCoordinates& coords)
    {
        vehicle_dynamics_.frenet_coords = coords;
        return *this;
    }

    /// @brief Build Data Source Object with Object Fusion
    DataSourceBuilder& WithObjectInLane(const GlobalLaneId global_lane_id,
                                        const units::velocity::meters_per_second_t velocity)
    {
        this->WithSensorFusion(SensorFusionBuilder()
                                   .WithObjectFusion(ObjectFusionBuilder()
                                                         .WithIndex(1)
                                                         .WithFrenetCoordinates(GetCoordinates(global_lane_id))
                                                         .WithVelocity(velocity)
                                                         .Build())
                                   .Build());
        return *this;
    }

    /// @brief Build Data Source Object with Distance Travelled (Ego)
    DataSourceBuilder& WithDistance(const units::length::meter_t longitudinal_distance)
    {
        previous_path_end_frenet_.s = longitudinal_distance.value();
        vehicle_dynamics_.frenet_coords.s = longitudinal_distance.value();
        return *this;
    }

    /// @brief Build Data Source Object with Fake Previous Path with number of Waypoints
    DataSourceBuilder& WithFakePreviousPath(const std::int32_t n_wp)
    {
        previous_path_global_.resize(n_wp);
        std::fill(previous_path_global_.begin(), previous_path_global_.end(), GlobalCoordinates{});
        return *this;
    }

    /// @brief Build Data Source Object with Previous Path End Position (Frenet Coordinates)
    DataSourceBuilder& WithPreviousPathEnd(const FrenetCoordinates& coords)
    {
        previous_path_end_frenet_ = coords;
        return *this;
    }

    /// @brief Build Data Source Object with provided Vehicle Dynamics
    DataSourceBuilder& WithVehicleDynamics(const VehicleDynamics& vehicle_dynamics)
    {
        vehicle_dynamics_ = vehicle_dynamics;
        return *this;
    }

    /// @brief Build Data Source Object with provided Sensor Fusion
    DataSourceBuilder& WithSensorFusion(const SensorFusion& sensor_fusion)
    {
        sensor_fusion_ = sensor_fusion;
        return *this;
    }

    /// @brief Build Data Source Object with provided Map Coordinates
    DataSourceBuilder& WithMapCoordinates(const std::vector<MapCoordinates>& map_coords)
    {
        map_coords_ = map_coords;
        return *this;
    }

    /// @brief Build Data Source Object with provided Previous Path (Global Coordinates)
    DataSourceBuilder& WithPreviousPath(const PreviousPathGlobal& previous_path_global)
    {
        previous_path_global_ = previous_path_global;
        return *this;
    }

    /// @brief Build Data Source Object with provided Speed Limit
    DataSourceBuilder& WithSpeedLimit(const units::velocity::meters_per_second_t speed_limit)
    {
        speed_limit_ = speed_limit;
        return *this;
    }

    /// @brief Build Data Source Object
    DataSource Build()
    {
        data_source_.SetPreviousPath(previous_path_global_);
        data_source_.SetPreviousPathEnd(previous_path_end_frenet_);
        data_source_.SetVehicleDynamics(vehicle_dynamics_);
        data_source_.SetSensorFusion(sensor_fusion_);
        data_source_.SetMapCoordinates(map_coords_);
        data_source_.SetSpeedLimit(speed_limit_);
        return data_source_;
    }

    /// @brief Convert Global Coordinates to Frenet Coordinates (uses Global Lane Information)
    static FrenetCoordinates GetCoordinates(const GlobalLaneId global_lane_id)
    {
        auto coords = FrenetCoordinates{24.0, 0.0};
        switch (global_lane_id)
        {
            case GlobalLaneId::kLeft:
            {
                coords.d = 2.0;
                break;
            }
            case GlobalLaneId::kCenter:
            {
                coords.d = 6.0;
                break;
            }
            case GlobalLaneId::kRight:
            {
                coords.d = 10.0;
                break;
            }
            case GlobalLaneId::kInvalid:
            default:
            {
                coords.d = 14.0;
                break;
            }
        }
        return coords;
    }

  private:
    /// @brief Vehicle Dynamics (Ego Information)
    VehicleDynamics vehicle_dynamics_;

    /// @brief Previous Path End Position (Frenet Coordinates)
    FrenetCoordinates previous_path_end_frenet_;

    /// @brief Previous Path Positions (Global Coordinates)
    PreviousPathGlobal previous_path_global_;

    /// @brief Map Coordinates (Global Coordinates)
    MapCoordinatesList map_coords_;

    /// @brief Sensor Fusion (Object Position, velocity, etc.) Information
    SensorFusion sensor_fusion_;

    /// @brief Speed Limit (meters per seconds)
    units::velocity::meters_per_second_t speed_limit_;

    /// @brief Data Source Object
    DataSource data_source_;
};
}  // namespace
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TEST_SUPPORT_BUILDERS_DATA_SOURCE_BUILDER_H
