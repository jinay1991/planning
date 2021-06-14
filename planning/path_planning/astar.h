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

///
/// @class AStar
/// @brief Provide A* based shortest path search functionality for given Graph
///
/// @tparam Graph [in] Graph Type to look in for finding shorest path using A*.
/// @tparam Location [in] Location Type for each node/cell in Graph represented in cartesian coordinate systems.
/// @tparam Cost [in] Cost Type for computing Cost for given node/cell while computing Shortest Path.
///
template <typename Graph, typename Location, typename Cost = double>
class AStar
{
  public:
    ///
    /// @brief Shortest Path (List of Locations)
    ///
    using ShortestPath = std::vector<Location>;

    ///
    /// @brief Constructor for A*
    ///
    /// @param graph [in] Instance to the Graph/Grid
    ///
    constexpr explicit AStar(const Graph& graph) : graph_{graph}, came_from_{}, cost_so_far_{}, shortest_path_{}
    {
        static_assert(std::is_pod<Location>::value, "Location must be plain type.");
    }

    ///
    /// @brief Search Shortest Path between Start and End Locations using A* and reconstruct shortest path.
    ///
    /// @param start [in] Start Location
    /// @param end [in] End Location
    ///
    constexpr void SearchShortestPath(const Location& start, const Location& end)
    {
        DetermineShortestPath(start, end);
        ReconstructShortestPath(start, end);
    }

    ///
    /// @brief Provide resultant shortest path (List of Locations)
    ///
    /// @return shortest path (list of Locations)
    ///
    constexpr const ShortestPath& GetShortestPath() const { return shortest_path_; }

  protected:
    ///
    /// @brief Provide Heuristic Cost for given Locations (from & to).
    /// @note Heuristic Cost = Euclidean Distance between those Locations (from & to)
    ///
    /// @param from [in] Location from for calculating Heuristic Cost
    /// @param to [in] Location to for calculating Heuristic Cost
    ///
    /// @return Computed Cost for given Locations.
    ///
    virtual Cost GetHeuristicCost(const Location& from, const Location& to) const
    {
        const units::length::meter_t euclidean_distance =
            units::math::abs(from.x - to.x) + units::math::abs(from.y - to.y);
        return euclidean_distance.to<Cost>();
    }

  private:
    ///
    /// @brief Determine Shortest Path between start and end Locations on Graph/Grid using A* algorithm.
    ///        Calculated Cost to the current node is calculated with f(x) = g(x) + h(x), where g(x) is the cost
    ///        associated to node in the Graph/Grid and h(x) is the Heuristic function. Uses Heuristic Function as
    ///        Euclidean distance between current and end Locations on Grid. Uses priority_queue for storing and rating
    ///        the visited nodes to find shortest path.
    ///
    /// @param start [in] Start Location on Graph/Grid
    /// @param end [in] End Location on Graph/Grid
    ///
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

    ///
    /// @brief Reconstructs the Path from already analyzed Graph/Grid using A* by preparing the list of Locations which
    /// can be joined together to represent the Shortest Path.
    ///
    /// @param start [in] Start Location on Graph/Grid
    /// @param end [in] End Location on Graph/Grid
    ///
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

    ///
    /// @brief Instance of Graph/Grid.
    /// @warning Graph must have method GetNeighbors
    ///
    const Graph& graph_;

    ///
    /// @brief Location Map for all the visited nodes/cells on Graph/Grid with it's relation to previous node/cell.
    ///
    std::unordered_map<Location, Location> came_from_;

    ///
    /// @brief Location to Cost Map for all the visited nodes/cells on Graph/Grid.
    ///
    std::unordered_map<Location, Cost> cost_so_far_;

    ///
    /// @brief Shortest Path found using A* in form of List of Locations.
    ///
    ShortestPath shortest_path_;
};
}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_ASTAR_H
