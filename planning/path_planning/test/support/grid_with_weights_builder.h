///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_TEST_SUPPORT_GRID_WITH_WEIGHTS_BUILDER_H
#define PLANNING_PATH_PLANNING_TEST_SUPPORT_GRID_WITH_WEIGHTS_BUILDER_H

#include "planning/path_planning/test/support/grid_location.h"
#include "planning/path_planning/test/support/grid_with_weights.h"

#include <units.h>

#include <initializer_list>

namespace planning
{

class GridWithWeightsBuilder
{
  public:
    explicit GridWithWeightsBuilder(const units::length::meter_t width, const units::length::meter_t height);
    GridWithWeightsBuilder& WithForest(const GridLocation& location);
    GridWithWeightsBuilder& WithForestList(const std::initializer_list<GridLocation>& location_list);
    GridWithWeightsBuilder& WithBlock(const GridLocation& start, const GridLocation& end);
    const GridWithWeights& Build() const;

  private:
    GridWithWeights grid_;
};

}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_TEST_SUPPORT_GRID_WITH_WEIGHTS_BUILDER_H
