///
/// @file
///
#ifndef MOTION_PLANNING_DOMAIN_MODEL_LANE_H_
#define MOTION_PLANNING_DOMAIN_MODEL_LANE_H_

#include <cstdint>
#include <iostream>
#include <string>

namespace motion_planning
{
struct LaneInformation
{
    enum class LaneId : std::int32_t
    {
        kLeft = 0,
        kEgo = 1,
        kRight = 2
    };

    LaneId id;

    bool drivable{false};

    double cost{0.0};
};

inline std::ostream& operator<<(std::ostream& out, const LaneInformation::LaneId& lane_id)
{
    switch (lane_id)
    {
        case LaneInformation::LaneId::kEgo:
            return out << "kEgo";
        case LaneInformation::LaneId::kRight:
            return out << "kRight";
        case LaneInformation::LaneId::kLeft:
            return out << "kLeft";
        default:
            return out << "";
    }
}

}  // namespace motion_planning

#endif  /// MOTION_PLANNING_DOMAIN_MODEL_LANE_H_
