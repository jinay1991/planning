///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_TEST_SUPPORT_SIMPLE_GRAPH_H
#define PLANNING_PATH_PLANNING_TEST_SUPPORT_SIMPLE_GRAPH_H

#include <limits>
#include <unordered_map>
#include <vector>

namespace planning
{

template <typename T>
class SimpleGraph
{
  public:
    SimpleGraph() : edges_{} {}

    void AddEdge(const T& node, const std::vector<T>& edge) { edges_[node] = edge; }

    std::vector<T> GetNeighbors(const T& id) const { return edges_[id]; }

    double GetCost(const T& from, const T& to) const { return std::numeric_limits<T>::min(); }

  private:
    std::unordered_map<T, std::vector<T>> edges_;
};

}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_TEST_SUPPORT_SIMPLE_GRAPH_H
