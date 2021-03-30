///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_DIJKSTRA_H
#define PLANNING_PATH_PLANNING_DIJKSTRA_H

#include "planning/path_planning/astar.h"

namespace planning
{

template <typename Graph, typename Location, typename Cost = double>
class Dijkstra : public AStar<Graph, Location, Cost>
{
  public:
    constexpr explicit Dijkstra(const Graph& graph) : AStar<Graph, Location, Cost>{graph} {}

  protected:
    constexpr Cost GetHeuristicCost(const Location& from, const Location& to) const override { return 0.0; }
};
}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_DIJKSTRA_H
