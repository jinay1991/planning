///
/// @file
/// @brief Contains builder utility for Sensor Fusion and Object Fusion Objects built-up.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_TEST_SENSOR_FUSION_BUILDER_H
#define PLANNING_MOTION_PLANNING_TEST_SENSOR_FUSION_BUILDER_H

#include "planning/datatypes/sensor_fusion.h"
#include "planning/datatypes/vehicle_dynamics.h"
#include "planning/motion_planning/test/support/object_fusion_builder.h"

namespace planning
{

/// @brief Sensor Fusion Object Builder
class SensorFusionBuilder
{
  public:
    /// @brief Constructor
    SensorFusionBuilder() : sensor_fusion_{} {}

    /// @brief Build Sensor Fusion object with provided Object Fusion information
    SensorFusionBuilder& WithObjectFusion(const ObjectFusion& object_fusion)
    {
        sensor_fusion_.objs.push_back(object_fusion);
        return *this;
    }

    /// @brief Build Sensor Fusion Object
    SensorFusion Build() const { return sensor_fusion_; }

  private:
    /// @brief Sensor Fusion Object
    SensorFusion sensor_fusion_;
};
}  // namespace planning

#endif  /// PLANNING_MOTION_PLANNING_TEST_SENSOR_FUSION_BUILDER_H
