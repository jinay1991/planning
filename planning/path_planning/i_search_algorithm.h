///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_I_SEARCH_ALGORITHM_H
#define PLANNING_PATH_PLANNING_I_SEARCH_ALGORITHM_H

#include "planning/datatypes/position.h"

#include <list>

namespace planning
{

class ISearchAlgorithm
{
  public:
    using ShortestPath = std::list<INode>;

    virtual ~IPlanner() = default;

    virtual void SearchShortestPath(const Position& start, const Position& end) = 0;
    virtual const ShortestPath& GetShortestPath() const = 0;
};

}  // namespace planning

#endif  /// PLANNING_PATH_PLANNING_I_SEARCH_ALGORITHM_H
