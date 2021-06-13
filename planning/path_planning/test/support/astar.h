///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_TEST_SUPPORT_ASTAR_H
#define PLANNING_PATH_PLANNING_TEST_SUPPORT_ASTAR_H

#include <units.h>

#include <algorithm>
#include <array>
#include <initializer_list>
#include <list>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace planning
{

using namespace units::literals;

template <typename T>
class SimpleGraph
{
  public:
    SimpleGraph() : edges_{} {}

    void AddEdge(const T& node, const std::vector<T>& edge) { edges_[node] = edge; }
    std::vector<T> GetNeighbors(const T& id) const { return edges_[id]; }
    double GetCost(const T& from, const T& to) const { return 0.0; }

  private:
    std::unordered_map<T, std::vector<T>> edges_;
};

template <typename T>
class SimpleGraphBuilder
{
  public:
    SimpleGraphBuilder() : graph_{} {}

    SimpleGraphBuilder& WithEdge(const T& node, const std::vector<T>& edge)
    {
        graph_.AddEdge(node, edge);
        return *this;
    }

    const SimpleGraph<T>& Build() const { return graph_; }

  private:
    SimpleGraph<T> graph_;
};

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

/// =============
/// HASH FUNCTION
/// =============
namespace std
{
/* implement hash function so we can put GridLocation into an unordered_set */
template <>
struct hash<planning::GridLocation>
{
    typedef planning::GridLocation argument_type;
    typedef std::size_t result_type;
    std::size_t operator()(const planning::GridLocation& id) const noexcept
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

class GridWithWeights : public SquareGrid
{
  public:
    explicit GridWithWeights(const units::length::meter_t width, const units::length::meter_t height)
        : SquareGrid{width, height}
    {
    }

    void AddForest(const GridLocation& location) { forests_.insert(location); }

    double GetCost(const GridLocation& from, const GridLocation& to) const
    {
        return ((forests_.find(to) != forests_.end()) ? 5.0 : 1.0);
    }

  private:
    std::unordered_set<GridLocation> forests_;
};

class SquareGridBuilder
{
  public:
    explicit SquareGridBuilder(const units::length::meter_t width, const units::length::meter_t height)
        : grid_{width, height}
    {
    }

    SquareGridBuilder& WithBlock(const GridLocation& start, const GridLocation& end)
    {
        grid_.AddBlock(start, end);
        return *this;
    }

    const SquareGrid& Build() const { return grid_; }

  private:
    SquareGrid grid_;
};

class GridWithWeightsBuilder
{
  public:
    explicit GridWithWeightsBuilder(const units::length::meter_t width, const units::length::meter_t height)
        : grid_{width, height}
    {
    }

    GridWithWeightsBuilder& WithForest(const GridLocation& location)
    {
        grid_.AddForest(location);
        return *this;
    }

    GridWithWeightsBuilder& WithForestList(const std::initializer_list<GridLocation>& location_list)
    {
        for (auto& location : location_list)
        {
            grid_.AddForest(location);
        }
        return *this;
    }

    GridWithWeightsBuilder& WithBlock(const GridLocation& start, const GridLocation& end)
    {
        grid_.AddBlock(start, end);
        return *this;
    }

    const GridWithWeights& Build() const { return grid_; }

  private:
    GridWithWeights grid_;
};


}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_TEST_SUPPORT_ASTAR_H
