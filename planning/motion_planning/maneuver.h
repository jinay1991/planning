///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_MOTION_PLANNING_MANEUVER_H
#define PLANNING_MOTION_PLANNING_MANEUVER_H

#include "planning/motion_planning/i_maneuver.h"

namespace planning
{
/// @brief Maneuver
class Maneuver : public IManeuver
{
  public:
    /// @brief Constructor. Default Constructor with default LaneId and target velocity.
    Maneuver();

    /// @brief Constructor. Initialize Maneuver with LaneId and target velocity
    explicit Maneuver(const LaneId& lane_id, const units::velocity::meters_per_second_t& velocity);

    /// @brief Get LaneId for Maneuver
    LaneId GetLaneId() const override;

    /// @brief Get Target Velocity for Maneuver
    units::velocity::meters_per_second_t GetVelocity() const override;

  private:
    /// @brief Maneuver LaneId
    LaneId lane_id_;

    /// @brief Maneuver Target Velocity
    units::velocity::meters_per_second_t velocity_;
};

/// @brief Comparator for Maneuvers
inline bool operator==(const Maneuver& lhs, const Maneuver& rhs) noexcept
{
    return ((lhs.GetLaneId() == rhs.GetLaneId()) && (lhs.GetVelocity() == rhs.GetVelocity()));
}

/// @brief String Stream for Maneuver information (used for printing verbose information)
inline std::ostream& operator<<(std::ostream& out, const Maneuver& maneuver)
{
    return out << "Maneuver{lane: " << maneuver.GetLaneId() << ", velocity: " << maneuver.GetVelocity() << "}";
}
}  // namespace planning
#endif  /// PLANNING_MOTION_PLANNING_MANEUVER_H
