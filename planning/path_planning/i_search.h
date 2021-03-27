///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_I_SEARCH_H
#define PLANNING_PATH_PLANNING_I_SEARCH_H

#include "planning/datatypes/path_planning.h"

#include <cstdint>
#include <vector>

namespace planning
{
class ISearch
{
  public:
    virtual ~ISearch() = default;

    virtual void SearchShortestPath(const Node& start, const Node& end) = 0;
    virtual const Path& GetShortestPath() = 0;
};

}  // namespace planning

#endif  /// PLANNING_PATH_PLANNING_I_SEARCH_H
