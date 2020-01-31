///
/// @file data_source_builder.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef DATA_SOURCE_BUILDER_H_
#define DATA_SOURCE_BUILDER_H_

#include <units.h>
#include <memory>

#include "motion_planning/domain_model/lane.h"
#include "motion_planning/domain_model/sensor_fusion.h"
#include "motion_planning/domain_model/vehicle_dynamics.h"
#include "motion_planning/roadmodel_data_source.h"
#include "sensor_fusion_builder.h"

using namespace motion_planning;

namespace
{
class DataSourceBuilder
{
  public:
    DataSourceBuilder() : data_source_{std::make_shared<RoadModelDataSource>()}
    {
        this->WithFakePreviousPath(50)
            .WithDistance(units::length::meter_t{24.0})
            .WithGlobalLaneId(GlobalLaneId::kCenter)
            .WithVelocity(units::velocity::meters_per_second_t{17.0})
            .WithSpeedLimit(units::velocity::meters_per_second_t{22.12848});
    }

    DataSourceBuilder& WithGlobalLaneId(const GlobalLaneId& global_lane_id)
    {
        const auto lateral_distance = units::length::meter_t{GetCoordinates(global_lane_id).d};
        previous_path_end_frenet_.d = lateral_distance.value();
        vehicle_dynamics_.frenet_coords.d = lateral_distance.value();
        vehicle_dynamics_.global_lane_id = global_lane_id;
        return *this;
    }

    DataSourceBuilder& WithVelocity(const units::velocity::meters_per_second_t& velocity)
    {
        vehicle_dynamics_.velocity = velocity;
        return *this;
    }

    DataSourceBuilder& WithFrenetCoordinates(const FrenetCoordinates& coords)
    {
        vehicle_dynamics_.frenet_coords = coords;
        return *this;
    }

    DataSourceBuilder& WithObjectInLane(const GlobalLaneId& global_lane_id,
                                        const units::velocity::meters_per_second_t& velocity)
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

    DataSourceBuilder& WithDistance(const units::length::meter_t& longitudinal_distance)
    {
        previous_path_end_frenet_.s = longitudinal_distance.value();
        vehicle_dynamics_.frenet_coords.s = longitudinal_distance.value();
        return *this;
    }

    DataSourceBuilder& WithFakePreviousPath(const std::int32_t n_wp)
    {
        previous_path_global_.resize(n_wp);
        std::fill(previous_path_global_.begin(), previous_path_global_.end(), GlobalCoordinates{});
        return *this;
    }

    DataSourceBuilder& WithPreviousPathEnd(const FrenetCoordinates& coords)
    {
        previous_path_end_frenet_ = coords;
        return *this;
    }

    DataSourceBuilder& WithVehicleDynamics(const VehicleDynamics& vehicle_dynamics)
    {
        vehicle_dynamics_ = vehicle_dynamics;
        return *this;
    }

    DataSourceBuilder& WithSensorFusion(const SensorFusion& sensor_fusion)
    {
        sensor_fusion_ = sensor_fusion;
        return *this;
    }

    DataSourceBuilder& WithMapCoordinates(const std::vector<MapCoordinates>& map_coords)
    {
        map_coords_ = map_coords;
        return *this;
    }

    DataSourceBuilder& WithPreviousPath(const PreviousPathGlobal& previous_path_global)
    {
        previous_path_global_ = previous_path_global;
        return *this;
    }
    DataSourceBuilder& WithSpeedLimit(const units::velocity::meters_per_second_t& speed_limit)
    {
        speed_limit_ = speed_limit;
        return *this;
    }
    std::shared_ptr<IDataSource>& Build()
    {
        data_source_->SetPreviousPath(previous_path_global_);
        data_source_->SetPreviousPathEnd(previous_path_end_frenet_);
        data_source_->SetVehicleDynamics(vehicle_dynamics_);
        data_source_->SetSensorFusion(sensor_fusion_);
        data_source_->SetMapCoordinates(map_coords_);
        data_source_->SetSpeedLimit(speed_limit_);
        return data_source_;
    }

    static const FrenetCoordinates GetCoordinates(const GlobalLaneId& global_lane_id)
    {
        auto coords = FrenetCoordinates{24.0, 0.0};
        switch (global_lane_id)
        {
            case GlobalLaneId::kLeft:
                coords.d = 2;
                break;
            case GlobalLaneId::kCenter:
                coords.d = 6;
                break;
            case GlobalLaneId::kRight:
                coords.d = 10;
                break;
            case GlobalLaneId::kInvalid:
            default:
                coords.d = 14;
                break;
        }
        return coords;
    }

  private:
    VehicleDynamics vehicle_dynamics_{};
    FrenetCoordinates previous_path_end_frenet_{};
    PreviousPathGlobal previous_path_global_{};
    MapCoordinatesList map_coords_{};
    SensorFusion sensor_fusion_{};
    units::velocity::meters_per_second_t speed_limit_{};

    std::shared_ptr<IDataSource> data_source_;
};

}  // namespace

#endif  /// DATA_SOURCE_BUILDER_H_
