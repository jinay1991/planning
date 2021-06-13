///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/test/support/square_grid.h"

#include <algorithm>

namespace planning
{
using namespace units::literals;

SquareGrid::SquareGrid(const units::length::meter_t width, const units::length::meter_t height)
    : width_{width},
      height_{height},
      blocks_{},
      directions_{GridLocation{1_m, 0_m}, GridLocation{-1_m, 0_m}, GridLocation{0_m, -1_m}, GridLocation{0_m, 1_m}}
{
    static_assert(std::tuple_size<Directions>::value == 4U, "Neighboring directions other than 4 is not supported.");
}

void SquareGrid::AddBlock(const GridLocation& start, const GridLocation& end)
{
    for (auto x = start.x; x < end.x; ++x)
    {
        for (auto y = start.y; y < end.y; ++y)
        {
            blocks_.insert(GridLocation{x, y});
        }
    }
}

SquareGrid::Neighbors SquareGrid::GetNeighbors(GridLocation id) const
{
    Neighbors neighbors;

    for (auto& dir : directions_)
    {
        const GridLocation next{id.x + dir.x, id.y + dir.y};
        if (IsInside(next) && (!IsBlocked(next)))
        {
            neighbors.push_back(next);
        }
    }

    if ((id.x.to<std::int32_t>() + id.y.to<std::int32_t>()) % 2 == 0)
    {
        // see "Ugly paths" section for an explanation:
        std::reverse(neighbors.begin(), neighbors.end());
    }

    return neighbors;
}

units::length::meter_t SquareGrid::GetHeight() const
{
    return height_;
}
units::length::meter_t SquareGrid::GetWidth() const
{
    return width_;
}

bool SquareGrid::IsInside(const GridLocation& id) const noexcept
{
    return (0.0_m <= id.x && id.x < width_ && 0.0_m <= id.y && id.y < height_);
}

bool SquareGrid::IsBlocked(GridLocation id) const
{
    return (blocks_.find(id) != blocks_.end());
}

}  // namespace planning
