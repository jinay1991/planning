///
/// @file sensor_fusion_builder.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef SENSOR_FUSION_BUILDER_H_
#define SENSOR_FUSION_BUILDER_H_

#include <units.h>
#include <cstdint>

#include "motion_planning/domain_model/sensor_fusion.h"
#include "motion_planning/domain_model/vehicle_dynamics.h"

namespace motion_planning
{
class ObjectFusionBuilder
{
  public:
    ObjectFusionBuilder() {}
    ObjectFusionBuilder& WithIndex(const std::int32_t idx)
    {
        object_fusion_.idx = idx;
        return *this;
    }
    ObjectFusionBuilder& WithGlobalCoordinates(const GlobalCoordinates& coords)
    {
        object_fusion_.global_coords = coords;
        return *this;
    }
    ObjectFusionBuilder& WithFrenetCoordinates(const FrenetCoordinates& coords)
    {
        object_fusion_.frenet_coords = coords;
        return *this;
    }

    ObjectFusionBuilder& WithVelocity(const units::velocity::meters_per_second_t& velocity)
    {
        object_fusion_.velocity = velocity;
        return *this;
    }

    ObjectFusion Build() const { return object_fusion_; }

  private:
    ObjectFusion object_fusion_{};
};

class SensorFusionBuilder
{
  public:
    SensorFusionBuilder() {}

    SensorFusionBuilder& WithObjectFusion(const ObjectFusion& object_fusion)
    {
        sensor_fusion_.objs.push_back(object_fusion);
        return *this;
    }

    SensorFusion Build() const { return sensor_fusion_; }

  private:
    SensorFusion sensor_fusion_{};
};

}  // namespace motion_planning

#endif  /// SENSOR_FUSION_BUILDER_H_
