///
/// @file roadmodel_data_source.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H_
#define PLANNING_MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H_

#include "planning/motion_planning/i_data_source.h"

namespace planning
{
/// @brief DataSource based on RoadModel (contains information from Front Sensors and Rear Sensors).
class RoadModelDataSource : public IDataSource
{
  public:
    /// @brief Constructor. Initialize with default values for all information.
    RoadModelDataSource();

    /// @brief Destructor.
    ~RoadModelDataSource() override;

    /// @brief Set current Vehicle Dynamics
    virtual void SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics) override;

    /// @brief Set Map Points
    virtual void SetMapCoordinates(const MapCoordinatesList& map_coordinates) override;

    /// @brief Set Previous Path Points for Hysteresis
    virtual void SetPreviousPath(const PreviousPathGlobal& previous_path_global) override;

    /// @brief Set Previous Path End (Last point of previous trajectory)
    virtual void SetPreviousPathEnd(const FrenetCoordinates& coords) override;

    /// @brief Set SensorFusion (Objects)
    virtual void SetSensorFusion(const SensorFusion& sensor_fusion) override;

    /// @brief Set Speed Limit (Traffic Rules)
    virtual void SetSpeedLimit(const units::velocity::meters_per_second_t& speed_limit) override;

    /// @brief Get Global LaneId based on provided Frenet Coordinates
    virtual GlobalLaneId GetGlobalLaneId(const FrenetCoordinates& coords) const override;

    /// @brief Get Global LaneId based for ego vehicle
    virtual GlobalLaneId GetGlobalLaneId() const override;

    /// @brief Get Previous Path End (Last point of previous trajectory)
    virtual FrenetCoordinates GetPreviousPathEnd() const override;

    /// @brief Get Vehicle Dynamics
    virtual VehicleDynamics GetVehicleDynamics() const override;

    /// @brief Get Map Points
    virtual MapCoordinatesList GetMapCoordinates() const override;

    /// @brief Get Previous Path Points in Global Coordinates
    virtual PreviousPathGlobal GetPreviousPathInGlobalCoords() const override;

    /// @brief Get SensorFusion (Objects)
    virtual SensorFusion GetSensorFusion() const override;

    /// @brief Get Speed Limit (Traffic Rules)
    virtual units::velocity::meters_per_second_t GetSpeedLimit() const override;

  private:
    /// @brief Check if given Frenet Coordinate is on Left Lane (Global)
    virtual bool IsLeftLane(const FrenetCoordinates& coords) const;

    /// @brief Check if given Frenet Coordinate is on Center Lane (Global)
    virtual bool IsCenterLane(const FrenetCoordinates& coords) const;

    /// @brief Check if given Frenet Coordinate is on Right Lane (Global)
    virtual bool IsRightLane(const FrenetCoordinates& coords) const;

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

#endif  /// PLANNING_MOTION_PLANNING_ROADMODEL_DATA_SOURCE_H_
