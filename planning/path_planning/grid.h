///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_GRID_H
#define PLANNING_PATH_PLANNING_GRID_H

#include "planning/datatypes/position.h"

#include <units.h>

#include <algorithm>
#include <list>
#include <unordered_set>

namespace planning
{

class Grid
{
  public:
    constexpr std::size_t kMaxDirection = 4U;
    using Neighbors = std::list<Position>;
    using ClosedPositions = std::unordered_set<Position>;
    using Directions = std::array<Position, kMaxDirection>;

    explicit Grid(const units::length::meter_t width, const units::length::meter_t height);
    const Neighbors& GetNeighbors(const Position& position);

  private:
    void UpdateNeighbors(const Position& position);

    bool IsInsideGrid(const Position& position) const;
    bool IsBlocked(const Position& position) const;

    units::length::meter_t width_;
    units::length::meter_t height_;

    Neighbors neighbors_;
    ClosedPositions closed_positions_;

    const Directions directions_;
};
}  // namespace planning

#endif  /// PLANNING_PATH_PLANNING_GRID_H
