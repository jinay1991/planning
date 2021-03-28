///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_GRID_GENERATE_H
#define PLANNING_PATH_PLANNING_GRID_GENERATE_H

#include <units.h>

#include <algorithm>
#include <array>
#include <list>
#include <unordered_set>
#include <vector>

namespace planning
{

using namespace units::literals;

struct GridLocation
{
    units::length::meter_t x;
    units::length::meter_t y;
};

inline bool operator==(const GridLocation& lhs, const GridLocation& rhs) noexcept
{
    return ((lhs.x == rhs.x) && (lhs.y == rhs.y));
}

inline bool operator!=(const GridLocation& lhs, const GridLocation& rhs) noexcept
{
    return !(lhs == rhs);
}

inline bool operator<(const GridLocation& lhs, const GridLocation& rhs) noexcept
{
    return ((lhs.x < rhs.x) && (lhs.y < rhs.y));
}

inline std::ostream& operator<<(std::ostream& out, const GridLocation& location) noexcept
{
    out << '(' << location.x << ',' << location.y << ')';
    return out;
}
}  // namespace planning
namespace std
{

/* implement hash function so we can put GridLocation into an unordered_set */
struct hash<GridLocation>
{
    typedef GridLocation argument_type;
    typedef std::size_t result_type;
    std::size_t operator()(const GridLocation& id) const noexcept
    {
        return (std::hash<std::int32_t>()(id.x.to<std::int32_t>() ^ (id.y.to<std::int32_t>() << 4)));
    }
};
}  // namespace std

namespace planning
{

class SquareGrid
{
  public:
    static constexpr std::size_t kMaxDirection = 4U;

    using Directions = std::array<GridLocation, kMaxDirection>;
    using Neighbors = std::vector<GridLocation>;
    using Blocks = std::unordered_set<GridLocation>;

    explicit SquareGrid(const units::length::meter_t width, const units::length::meter_t height)
        : width_{width},
          height_{height},
          blocks_{},
          directions_{GridLocation{1_m, 0_m}, GridLocation{-1_m, 0_m}, GridLocation{0_m, -1_m}, GridLocation{0_m, 1_m}}
    {
        static_assert(std::tuple_size<Directions>::value == 4U,
                      "Neighboring directions other than 4 is not supported.");
    }

    void AddBlock(const GridLocation& start, const GridLocation& end)
    {
        for (auto x = start.x; x < end.x; ++x)
        {
            for (auto y = start.y; y < end.y; ++y)
            {
                blocks_.insert(GridLocation{x, y});
            }
        }
    }

    Neighbors GetNeighbors(GridLocation id) const
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

    units::length::meter_t GetHeight() const { return height_; }
    units::length::meter_t GetWidth() const { return width_; }

    inline bool IsInside(const GridLocation& id) const noexcept
    {
        return (0.0_m <= id.x && id.x < width_ && 0.0_m <= id.y && id.y < height_);
    }

    inline bool IsBlocked(GridLocation id) const { return (blocks_.find(id) != blocks_.end()); }

  private:
    units::length::meter_t width_;
    units::length::meter_t height_;

    Blocks blocks_;

    const Directions directions_;
};

}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_GRID_GENERATE_H
