///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#include "planning/path_planning/astar_search_algorithm.h"

namespace planning
{

AStarSearchAlgorithm::AStarSearchAlgorithm() : grid_{}, shortest_path_{} {}

void AStarSearchAlgorithm::SearchShortestPath(const Position& start, const Position& end)
{
    shortest_path_.insert(start);
}

const ShortestPath& AStarSearchAlgorithm::GetShortestPath() const
{
    return shortest_path_;
}

}  // namespace planning
