///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_BREADTH_FIRST_SEARCH_H
#define PLANNING_PATH_PLANNING_BREADTH_FIRST_SEARCH_H

#include <algorithm>
#include <queue>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace planning
{

///
/// @class BreadthFirstSearch
/// @brief Provide BFS (Breadth First Search) based shortest path search functionality for given Graph
///
/// @tparam Graph [in] Graph Type to look in for finding shorest path using A*.
/// @tparam Location [in] Location Type for each node/cell in Graph represented in cartesian coordinate systems.
///
template <typename Graph, typename Location>
class BreadthFirstSearch
{
  public:
    ///
    /// @brief Shortest Path (List of Locations)
    ///
    using ShortestPath = std::vector<Location>;

    ///
    /// @brief Constructor for BreadthFirstSearch
    ///
    /// @param graph [in] Instance to the Graph/Grid
    ///
    constexpr explicit BreadthFirstSearch(const Graph& graph) : graph_{graph}, came_from_{}, shortest_path_{}
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

  private:
    ///
    /// @brief Determine Shortest Path between start and end Locations on Graph using Breadth First Search algorithm.
    ///
    /// @param start [in] Start Location on Graph/Grid
    /// @param end [in] End Location on Graph/Grid
    ///
    constexpr void DetermineShortestPath(const Location& start, const Location& end)
    {
        std::queue<Location> queue;
        queue.push(start);

        came_from_[start] = start;

        while (!queue.empty())
        {
            Location current = queue.front();
            queue.pop();

            if (current == end)
            {
                break;
            }

            for (Location next : graph_.GetNeighbors(current))
            {
                if (came_from_.find(next) == came_from_.end())
                {
                    queue.push(next);
                    came_from_[next] = current;
                }
            }
        }
    }

    ///
    /// @brief Reconstructs the Path from already analyzed Graph/Grid using Breadth First Search by preparing the list
    /// of Locations which can be joined together to represent the Shortest Path.
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
    /// @brief Shortest Path found using A* in form of List of Locations.
    ///
    ShortestPath shortest_path_;
};

}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_BREADTH_FIRST_SEARCH_H