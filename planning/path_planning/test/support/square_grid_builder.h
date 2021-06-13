///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_TEST_SUPPORT_SQUARE_GRID_BUILDER_H
#define PLANNING_PATH_PLANNING_TEST_SUPPORT_SQUARE_GRID_BUILDER_H

#include "planning/path_planning/test/support/grid_location.h"
#include "planning/path_planning/test/support/square_grid.h"

#include <units.h>

namespace planning
{

class SquareGridBuilder
{
  public:
    explicit SquareGridBuilder(const units::length::meter_t width, const units::length::meter_t height);
    SquareGridBuilder& WithBlock(const GridLocation& start, const GridLocation& end);
    const SquareGrid& Build() const;

  private:
    SquareGrid grid_;
};

}  // namespace planning
#endif  /// PLANNING_PATH_PLANNING_TEST_SUPPORT_SQUARE_GRID_BUILDER_H
