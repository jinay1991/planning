///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_ASTAR_H
#define PLANNING_PATH_PLANNING_ASTAR_H

#include <units.h>

#include <algorithm>
#include <queue>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace planning
{

template <typename Graph, typename Location, typename Cost = double>
class AStar
{
  public:
    using ShortestPath = std::vector<Location>;

    constexpr explicit AStar(const Graph& graph) : graph_{graph}, came_from_{}, cost_so_far_{}, shortest_path_{}
    {
        // static_assert(std::is_base_of<IGraph, Graph>::value, "Graph must be derived class of IGraph.");
        // static_assert(std::is_pod<Location>::value, "Location must be plain type.");
    }

    constexpr void SearchShortestPath(const Location& start, const Location& end)
    {
        DetermineShortestPath(start, end);
        ReconstructShortestPath(start, end);
    }

    constexpr const ShortestPath& GetShortestPath() const { return shortest_path_; }

  protected:
    virtual constexpr Cost GetHeuristicCost(const Location& from, const Location& to) const
    {
        return static_cast<double>((units::math::abs(from.x - to.x) + units::math::abs(from.y - to.y)).value());
    }

  private:
    constexpr void DetermineShortestPath(const Location& start, const Location& end)
    {
        using LocationCostPair = std::pair<Cost, Location>;
        std::priority_queue<LocationCostPair, std::vector<LocationCostPair>, std::greater<LocationCostPair>>
            priority_queue;
        priority_queue.emplace(0.0, start);

        came_from_[start] = start;
        cost_so_far_[start] = 0.0;

        while (!priority_queue.empty())
        {
            const auto current = priority_queue.top().second;
            priority_queue.pop();

            if (current == end)
            {
                break;
            }

            for (const auto& next : graph_.GetNeighbors(current))
            {
                const Cost current_cost = cost_so_far_[current] + graph_.GetCost(current, next);
                if ((cost_so_far_.find(next) == cost_so_far_.end()) || (current_cost < cost_so_far_[next]))
                {
                    cost_so_far_[next] = current_cost;
                    const Cost total_cost = current_cost + GetHeuristicCost(next, end);
                    priority_queue.emplace(total_cost, next);
                    came_from_[next] = current;
                }
            }
        }
    }

    constexpr void ReconstructShortestPath(const Location& start, const Location& end)
    {
        shortest_path_.clear();
        Location current = end;
        while (current != start)
        {
            shortest_path_.push_back(current);
            current = came_from_[current];
        }
        shortest_path_.push_back(start);
        std::reverse(shortest_path_.begin(), shortest_path_.end());
    }

    const Graph& graph_;

    std::unordered_map<Location, Location> came_from_;
    std::unordered_map<Location, Cost> cost_so_far_;
    ShortestPath shortest_path_;
};
}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_ASTAR_H
