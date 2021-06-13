///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_TEST_SUPPORT_GRID_WITH_WEIGHTS_H
#define PLANNING_PATH_PLANNING_TEST_SUPPORT_GRID_WITH_WEIGHTS_H

#include "planning/path_planning/test/support/grid_location.h"
#include "planning/path_planning/test/support/square_grid.h"

#include <units.h>

#include <array>
#include <unordered_set>
#include <vector>

namespace planning
{

class GridWithWeights : public SquareGrid
{
  public:
    explicit GridWithWeights(const units::length::meter_t width, const units::length::meter_t height);
    void AddForest(const GridLocation& location);
    double GetCost(const GridLocation& from, const GridLocation& to) const;

  private:
    std::unordered_set<GridLocation> forests_;
};

}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_TEST_SUPPORT_GRID_WITH_WEIGHTS_H
