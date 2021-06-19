///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_I_DATA_SOURCE_H
#define PLANNING_MOTION_PLANNING_I_DATA_SOURCE_H

#include "planning/datatypes/lane.h"
#include "planning/datatypes/sensor_fusion.h"
#include "planning/datatypes/trajectory.h"
#include "planning/datatypes/vehicle_dynamics.h"

namespace planning
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using LaneId = LaneInformation::LaneId;

/// @brief Interface for Data Source (Inputs from SensorFusion and MapPoints)
class IDataSource
{
  public:
    /// @brief Destructor
    virtual ~IDataSource() = default;

    /// @brief Set current Vehicle Dynamics
    virtual void SetVehicleDynamics(const VehicleDynamics& vehicle_dynamics) = 0;

    /// @brief Set Map Points
    virtual void SetMapCoordinates(const MapCoordinatesList& map_coordinates) = 0;

    /// @brief Set Previous Path Points for Hysteresis
    virtual void SetPreviousPath(const PreviousPathGlobal& previous_path_global) = 0;

    /// @brief Set Previous Path End (Last point of previous trajectory)
    virtual void SetPreviousPathEnd(const FrenetCoordinates& coords) = 0;

    /// @brief Set SensorFusion (Objects)
    virtual void SetSensorFusion(const SensorFusion& sensor_fusion) = 0;

    /// @brief Set Speed Limit (Traffic Rules)
    virtual void SetSpeedLimit(const units::velocity::meters_per_second_t speed_limit) = 0;

    /// @brief Get Global LaneId based on provided Frenet Coordinates
    virtual GlobalLaneId GetGlobalLaneId(const FrenetCoordinates& coords) const = 0;

    /// @brief Get Global LaneId based for ego vehicle
    virtual GlobalLaneId GetGlobalLaneId() const = 0;

    /// @brief Get Previous Path End (Last point of previous trajectory)
    virtual FrenetCoordinates GetPreviousPathEnd() const = 0;

    /// @brief Get Vehicle Dynamics
    virtual VehicleDynamics GetVehicleDynamics() const = 0;

    /// @brief Get Map Points
    virtual MapCoordinatesList GetMapCoordinates() const = 0;

    /// @brief Get Previous Path Points in Global Coordinates
    virtual PreviousPathGlobal GetPreviousPathInGlobalCoords() const = 0;

    /// @brief Get SensorFusion (Objects)
    virtual SensorFusion GetSensorFusion() const = 0;

    /// @brief Get Speed Limit (Traffic Rules)
    virtual units::velocity::meters_per_second_t GetSpeedLimit() const = 0;
};
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_I_DATA_SOURCE_H
