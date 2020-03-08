///
/// @file maneuver.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef MOTION_PLANNING_MANEUVER_H_
#define MOTION_PLANNING_MANEUVER_H_

#include "motion_planning/i_maneuver.h"

namespace motion_planning
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
    virtual LaneId GetLaneId() const override;

    /// @brief Get Target Velocity for Maneuver
    virtual units::velocity::meters_per_second_t GetVelocity() const override;

    /// @brief Comparator for Maneuvers
    inline bool operator==(const Maneuver& rhs) const { return lane_id_ == rhs.lane_id_ && velocity_ == rhs.velocity_; }

  private:
    /// @brief Maneuver LaneId
    LaneId lane_id_;

    /// @brief Maneuver Target Velocity
    units::velocity::meters_per_second_t velocity_;
};

/// @brief String Stream for Maneuver information (used for printing verbose information)
inline std::ostream& operator<<(std::ostream& out, const Maneuver& maneuver)
{
    return out << "Maneuver{lane: " << maneuver.GetLaneId() << ", velocity: " << maneuver.GetVelocity() << "}";
}
}  // namespace motion_planning
#endif  /// MOTION_PLANNING_MANEUVER_H_
