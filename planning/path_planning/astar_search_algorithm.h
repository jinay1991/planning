///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_ASTAR_SEARCH_ALGORITHM_H
#define PLANNING_PATH_PLANNING_ASTAR_SEARCH_ALGORITHM_H

#include "planning/path_planning/grid.h"
#include "planning/path_planning/i_search_algorithm.h"

namespace planning
{

class AStarSearchAlgorithm : public ISearchAlgorithm
{
  public:
    AStarSearchAlgorithm();

    void SearchShortestPath(const Position& start, const Position& end) override;

    const ShortestPath& GetShortestPath() const override;

  private:
    Grid grid_;
    ShortestPath shortest_path_;
};

}  // namespace planning

#endif  /// PLANNING_PATH_PLANNING_ASTAR_SEARCH_ALGORITHM_H
