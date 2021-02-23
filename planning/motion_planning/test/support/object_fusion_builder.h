///
/// @file
/// @brief Contains builder utility for Object Fusion Objects built-up.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_TEST_OBJECT_FUSION_BUILDER_H
#define PLANNING_MOTION_PLANNING_TEST_OBJECT_FUSION_BUILDER_H

#include "planning/datatypes/sensor_fusion.h"
#include "planning/datatypes/vehicle_dynamics.h"

#include <units.h>

#include <cstdint>

namespace planning
{
/// @brief Object Fusion Object Builder
class ObjectFusionBuilder
{
  public:
    /// @brief Constructor.
    ObjectFusionBuilder() : object_fusion_{} {}

    /// @brief Build Object Fusion object with Object Id
    ObjectFusionBuilder& WithIndex(const std::int32_t idx)
    {
        object_fusion_.idx = idx;
        return *this;
    }

    /// @brief Build Object Fusion object with Object Position (Global Coordinates)
    ObjectFusionBuilder& WithGlobalCoordinates(const GlobalCoordinates& coords)
    {
        object_fusion_.global_coords = coords;
        return *this;
    }

    /// @brief Build Object Fusion object with Object Position (Frenet Coordinates)
    ObjectFusionBuilder& WithFrenetCoordinates(const FrenetCoordinates& coords)
    {
        object_fusion_.frenet_coords = coords;
        return *this;
    }

    /// @brief Build Object Fusion object with Object Velocity (meters per seconds)
    ObjectFusionBuilder& WithVelocity(const units::velocity::meters_per_second_t& velocity)
    {
        object_fusion_.velocity = velocity;
        return *this;
    }

    /// @brief Build Object Fusion Object
    ObjectFusion Build() const { return object_fusion_; }

  private:
    /// @brief Object Fusion object
    ObjectFusion object_fusion_;
};
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_TEST_OBJECT_FUSION_BUILDER_H
