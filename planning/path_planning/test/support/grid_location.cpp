///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/test/support/grid_location.h"

#include <ostream>
#include <string>

namespace planning
{

bool operator==(const GridLocation& lhs, const GridLocation& rhs) noexcept
{
    return ((lhs.x == rhs.x) && (lhs.y == rhs.y));
}

bool operator!=(const GridLocation& lhs, const GridLocation& rhs) noexcept
{
    return !(lhs == rhs);
}

bool operator<(const GridLocation& lhs, const GridLocation& rhs) noexcept
{
    return ((lhs.x < rhs.x) && (lhs.y < rhs.y));
}

std::ostream& operator<<(std::ostream& out, const GridLocation& location) noexcept
{
    out << '(' << location.x << ',' << location.y << ')';
    return out;
}

}  // namespace planning
