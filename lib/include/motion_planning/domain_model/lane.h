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
    enum class LaneId : std::uint8_t
    {
        kLeft = 0,
        kEgo = 1,
        kRight = 2,
        kInvalid = 255
    };

    enum class GlobalLaneId : std::uint8_t
    {
        kLeft = 0,
        kCenter = 1,
        kRight = 2,
        kInvalid = 255
    };

    LaneId lane_id;
    GlobalLaneId global_lane_id;

    bool drivable{false};

    double cost{0.0};
};

inline LaneInformation::LaneId operator+(const LaneInformation::LaneId& lhs, const std::int32_t& offset)
{
    if (lhs < LaneInformation::LaneId::kRight)
    {
        return static_cast<LaneInformation::LaneId>(static_cast<std::int32_t>(lhs) + offset);
    }
    else
    {
        return LaneInformation::LaneId::kInvalid;
    }
}

inline LaneInformation::LaneId operator-(const LaneInformation::LaneId& lhs, const std::int32_t& offset)
{
    if (lhs > LaneInformation::LaneId::kLeft)
    {
        return static_cast<LaneInformation::LaneId>(static_cast<std::int32_t>(lhs) - offset);
    }
    else
    {
        return LaneInformation::LaneId::kInvalid;
    }
}
inline LaneInformation::GlobalLaneId operator+(const LaneInformation::GlobalLaneId& lhs, const std::int32_t& offset)
{
    if (lhs < LaneInformation::GlobalLaneId::kRight)
    {
        return static_cast<LaneInformation::GlobalLaneId>(static_cast<std::int32_t>(lhs) + offset);
    }
    else
    {
        return LaneInformation::GlobalLaneId::kInvalid;
    }
}

inline LaneInformation::GlobalLaneId operator-(const LaneInformation::GlobalLaneId& lhs, const std::int32_t& offset)
{
    if (lhs > LaneInformation::GlobalLaneId::kLeft)
    {
        return static_cast<LaneInformation::GlobalLaneId>(static_cast<std::int32_t>(lhs) - offset);
    }
    else
    {
        return LaneInformation::GlobalLaneId::kInvalid;
    }
}
inline std::ostream& operator<<(std::ostream& out, const LaneInformation::LaneId& lane_id)
{
    switch (lane_id)
    {
        case LaneInformation::LaneId::kEgo:
            return out << "LandId::kEgo";
        case LaneInformation::LaneId::kRight:
            return out << "LandId::kRight";
        case LaneInformation::LaneId::kLeft:
            return out << "LandId::kLeft";
        case LaneInformation::LaneId::kInvalid:
            return out << "LandId::kInvalid";
    }
}

inline std::ostream& operator<<(std::ostream& out, const LaneInformation::GlobalLaneId& lane_id)
{
    switch (lane_id)
    {
        case LaneInformation::GlobalLaneId::kCenter:
            return out << "GlobalLaneId::kCenter";
        case LaneInformation::GlobalLaneId::kRight:
            return out << "GlobalLaneId::kRight";
        case LaneInformation::GlobalLaneId::kLeft:
            return out << "GlobalLaneId::kLeft";
        case LaneInformation::GlobalLaneId::kInvalid:
            return out << "GlobalLaneId::kInvalid";
    }
}
}  // namespace motion_planning

#endif  /// MOTION_PLANNING_DOMAIN_MODEL_LANE_H_
