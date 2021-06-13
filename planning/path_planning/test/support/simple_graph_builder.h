///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_TEST_SUPPORT_SIMPLE_GRAPH_H
#define PLANNING_PATH_PLANNING_TEST_SUPPORT_SIMPLE_GRAPH_H

#include "planning/path_planning/test/support/simple_graph.h"

#include <vector>

namespace planning
{

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

}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_TEST_SUPPORT_SIMPLE_GRAPH_H
