///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#include "planning/path_planning/grid.h"

namespace planning
{
namespace
{
template <typename T>
constexpr bool IsInRangeExclusive(const T& value, const T& lower, const T& upper) noexcept
{
    return ((value > lower) && (value < upper));
}
}  // namespace

using namespace units::literals;

Grid::Grid(const units::length::meter_t width, const units::length::meter_t height)
    : width_{height},
      height_{height},
      neighbors_{},
      closed_positions_{},
      directions_{Position{1_m, 0_m, 0_m}, Position{-1_m, 0_m, 0_m}, Position{0_m, -1_m, 0_m}, Position{0_m, 1_m, 0_m}}
{
}

const Neighbors& Grid::GetNeighbors(const Position& position)
{
    UpdateNeighbors(position);
    return neighbors_;
}

void Grid::UpdateNeighbors(const Position& position)
{
    for (const auto& direction : directions)
    {
        const Position next_position{position.x + direction.x, position.y + direction.y};
        if (IsInsideGrid(next_position) && (!IsBlocked(next_position)))
        {
            neighbors_.insert(next_position);
        }
    }

    if ((position.x + position.y).to<std::int32_t>() % 2 == 0)
    {
        std::reverse(neighbors_.begin(), neighbors_.end());
    }
}

bool Grid::IsInsideGrid(const Position& position) const
{
    return (IsInRangeInclusive(position.x, 0.0_m, width_) && IsInRangeInclusive(position.y, 0.0_m, height_));
}

bool Grid::IsBlocked(const Position& position) const
{
    return (closed_positions_.find(position) != closed_positions_.end());
}

}  // namespace planning
