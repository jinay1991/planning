///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/test/support/astar.h"
#include "planning/path_planning/test/support/draw_grid.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

namespace planning
{
namespace
{
using namespace units::literals;

TEST(SquareGrid, AddBlock)
{
    // Given
    SquareGrid grid{30_m, 15_m};
    GridLocation start{3_m, 3_m};
    GridLocation end{5_m, 12_m};

    // When
    grid.AddBlock(start, end);

    // Then
    draw_grid(grid);
}
}  // namespace
}  // namespace planning
