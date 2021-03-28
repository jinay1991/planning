///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/test/support/astar.h"
#include "planning/path_planning/test/support/draw_grid.h"
#include "planning/path_planning/test/support/test_data.h"

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
    DrawGrid(grid);
}

TEST(AStarSearch, GivenTypicalGridDiagram_ExpectShortestPath)
{
    // Given
    std::unordered_map<GridLocation, GridLocation> came_from;
    std::unordered_map<GridLocation, double> cost_so_far;
    GridLocation start{0_m, 0_m};
    GridLocation end{30_m, 15_m};

    // When
    AStarSearch(kGridWithWeightsDiagram, start, end, came_from, cost_so_far);

    // Then
    DrawGrid(kGridWithWeightsDiagram);
    const auto shortest_path = GetReconstructPath<GridLocation>(start, end, came_from);
    EXPECT_GT(shortest_path.size(), 1U);
}
}  // namespace
}  // namespace planning
