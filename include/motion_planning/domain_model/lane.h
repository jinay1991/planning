///
/// @file
///
#ifndef MOTION_PLANNING_DOMAIN_MODEL_LANE_H_
#define MOTION_PLANNING_DOMAIN_MODEL_LANE_H_

#include <cstdint>

namespace motion_planning
{
struct LaneInformation
{
    enum class LaneId : std::int8_t
    {
        kRight = -1,
        kEgo = 0,
        kLeft = 1
    };

    LaneId id;

    bool drivable{false};

    double cost{0.0};
};

}  // namespace motion_planning

#endif  /// MOTION_PLANNING_DOMAIN_MODEL_LANE_H_