///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H
#define PLANNING_MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H

#include "planning/motion_planning/i_data_source.h"

namespace planning
{
/// @brief DataSource based on RoadModel (contains information from Front Sensors and Rear Sensors).
class DataSource : public IDataSource
{
  public:
    /// @brief Constructor. Initialize with default values for all information.
    DataSource();

    /// @brief Set current Vehicle Dynamics
    void SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics) override;

    /// @brief Set Map Points
    void SetMapCoordinates(const MapCoordinatesList& map_coordinates) override;

    /// @brief Set Previous Path Points for Hysteresis
    void SetPreviousPath(const PreviousPathGlobal& previous_path_global) override;

    /// @brief Set Previous Path End (Last point of previous trajectory)
    void SetPreviousPathEnd(const FrenetCoordinates& coords) override;

    /// @brief Set SensorFusion (Objects)
    void SetSensorFusion(const SensorFusion& sensor_fusion) override;

    /// @brief Set Speed Limit (Traffic Rules)
    void SetSpeedLimit(const units::velocity::meters_per_second_t& speed_limit) override;

    /// @brief Get Global LaneId based on provided Frenet Coordinates
    GlobalLaneId GetGlobalLaneId(const FrenetCoordinates& coords) const override;

    /// @brief Get Global LaneId based for ego vehicle
    GlobalLaneId GetGlobalLaneId() const override;

    /// @brief Get Previous Path End (Last point of previous trajectory)
    FrenetCoordinates GetPreviousPathEnd() const override;

    /// @brief Get Vehicle Dynamics
    VehicleDynamics GetVehicleDynamics() const override;

    /// @brief Get Map Points
    MapCoordinatesList GetMapCoordinates() const override;

    /// @brief Get Previous Path Points in Global Coordinates
    PreviousPathGlobal GetPreviousPathInGlobalCoords() const override;

    /// @brief Get SensorFusion (Objects)
    SensorFusion GetSensorFusion() const override;

    /// @brief Get Speed Limit (Traffic Rules)
    units::velocity::meters_per_second_t GetSpeedLimit() const override;

  private:
    /// @brief Check if given Frenet Coordinate is on Left Lane (Global)
    static bool IsLeftLane(const FrenetCoordinates& coords);

    /// @brief Check if given Frenet Coordinate is on Center Lane (Global)
    static bool IsCenterLane(const FrenetCoordinates& coords);

    /// @brief Check if given Frenet Coordinate is on Right Lane (Global)
    static bool IsRightLane(const FrenetCoordinates& coords);

    /// @brief Global Lane Id for Ego Vehicle
    GlobalLaneId global_lane_id_;

    /// @brief Current Vehicle Dynamics
    VehicleDynamics vehicle_dynamics_;

    /// @brief Map Points
    MapCoordinatesList map_coordinates_;

    /// @brief Previous Path Points (Global Coordinates)
    PreviousPathGlobal previous_path_global_;

    /// @brief Previous Path End (previous trajectory end point in Frenet Coordinate)
    FrenetCoordinates previous_path_end_frenet_;

    /// @brief Current SensorFusion Information (objects)
    SensorFusion sensor_fusion_;

    /// @brief Current Speed Limit
    units::velocity::meters_per_second_t speed_limit_;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H
