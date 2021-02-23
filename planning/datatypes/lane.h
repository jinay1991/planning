///
/// @file
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#ifndef PLANNING_DATATYPES_LANE_H_
#define PLANNING_DATATYPES_LANE_H_

#include <cstdint>
#include <ostream>
#include <string>

namespace planning
{
/// @brief LaneInformation
struct LaneInformation
{
    /// @brief LaneId
    enum class LaneId : std::uint8_t
    {
        kLeft = 0U,
        kEgo = 1U,
        kRight = 2U,
        kInvalid = 255U
    };

    /// @brief GlobalLaneId
    enum class GlobalLaneId : std::uint8_t
    {
        kLeft = 0U,
        kCenter = 1U,
        kRight = 2U,
        kInvalid = 255U
    };

    /// @brief LaneId (Local Coordinates)
    LaneId lane_id;

    /// @brief LaneId (Global Coordinates)
    GlobalLaneId global_lane_id;

    /// @brief Lane Drivability
    bool drivable{false};

    /// @brief Lane Cost
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
            return out << "LaneId::kEgo";
        case LaneInformation::LaneId::kRight:
            return out << "LaneId::kRight";
        case LaneInformation::LaneId::kLeft:
            return out << "LaneId::kLeft";
        case LaneInformation::LaneId::kInvalid:
        default:
            return out << "LaneId::kInvalid";
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
        default:
            return out << "GlobalLaneId::kInvalid";
    }
}
}  // namespace planning

#endif  /// PLANNING_DATATYPES_LANE_H_
