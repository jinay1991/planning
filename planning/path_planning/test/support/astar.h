///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_GRID_GENERATE_H
#define PLANNING_PATH_PLANNING_GRID_GENERATE_H

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

inline units::length::meter_t CalculateHeuristic(const GridLocation& from, const GridLocation& to)
{
    return (units::math::abs(from.x - to.x) + units::math::abs(from.y - to.y));
}

template <typename Location>
std::vector<Location> GetRecontructPath(const Location& start,
                                        const Location& end,
                                        const std::unordered_map<Location, Location>& came_from)
{
    std::vector<Location> path;
    Location current = end;
    while (current != start)
    {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

template <typename T, typename priority_t>
struct PriorityQueue
{
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

    inline bool empty() const { return elements.empty(); }

    inline void put(T item, priority_t priority) { elements.emplace(priority, item); }

    T get()
    {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

template <typename Location, typename Graph>
void DijkstraSearch(const Graph& graph,
                    const Location& start,
                    const Location& end,
                    std::unordered_map<Location, Location>& came_from,
                    std::unordered_map<Location, double>& cost_so_far)
{
    PriorityQueue<Location, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty())
    {
        Location current = frontier.get();

        if (current == end)
        {
            break;
        }

        for (Location next : graph.neighbors(current))
        {
            double new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
            {
                cost_so_far[next] = new_cost;
                came_from[next] = current;
                frontier.put(next, new_cost);
            }
        }
    }
}

template <typename Location, typename Graph>
void AStarSearch(const Graph& graph,
                 const Location& start,
                 const Location& end,
                 std::unordered_map<Location, Location>& came_from,
                 std::unordered_map<Location, double>& cost_so_far)
{
    PriorityQueue<Location, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty())
    {
        Location current = frontier.get();

        if (current == end)
        {
            break;
        }

        for (Location next : graph.neighbors(current))
        {
            double new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
            {
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(next, end);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }
}
}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_GRID_GENERATE_H
