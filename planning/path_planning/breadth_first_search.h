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

template <typename Graph, typename Location>
class BreadthFirstSearch
{
  public:
    using ShortestPath = std::vector<Location>;

    constexpr explicit BreadthFirstSearch(const Graph& graph) : graph_{graph}, came_from_{}, shortest_path_{}
    {
        static_assert(std::is_pod<Location>::value, "Location must be plain type.");
    }

    constexpr void SearchShortestPath(const Location& start, const Location& end)
    {
        DetermineShortestPath(start, end);
        ReconstructShortestPath(start, end);
    }

    constexpr const ShortestPath& GetShortestPath() const { return shortest_path_; }

  private:
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
    ShortestPath shortest_path_;
};

}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_BREADTH_FIRST_SEARCH_H