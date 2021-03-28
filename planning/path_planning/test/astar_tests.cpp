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

#include <type_traits>

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
    GridLocation end{8_m, 5_m};

    // When
    AStarSearch(kGridWithWeightsDiagram, start, end, came_from, cost_so_far);

    // Then
    auto shortest_path = GetReconstructPath<GridLocation>(start, end, came_from);
    DrawGrid(kGridWithWeightsDiagram, &cost_so_far, &came_from, &shortest_path, &start, &end);
    EXPECT_GT(shortest_path.size(), 1U);
}

TEST(DijkstraSearch, GivenTypicalGridDiagram_ExpectShortestPath)
{
    // Given
    std::unordered_map<GridLocation, GridLocation> came_from;
    std::unordered_map<GridLocation, double> cost_so_far;
    GridLocation start{0_m, 0_m};
    GridLocation end{8_m, 5_m};

    // When
    DijkstraSearch(kGridWithWeightsDiagram, start, end, came_from, cost_so_far);

    // Then
    auto shortest_path = GetReconstructPath<GridLocation>(start, end, came_from);
    DrawGrid(kGridWithWeightsDiagram, &cost_so_far, &came_from, &shortest_path, &start, &end);
    EXPECT_GT(shortest_path.size(), 1U);
}

TEST(BreadthFirstSearch, GivenTypicalGridDiagram_ExpectShortestPath)
{
    // Given
    std::unordered_map<GridLocation, GridLocation> came_from;
    GridLocation start{0_m, 0_m};
    GridLocation end{8_m, 5_m};

    // When
    BreadthFirstSearch(kSquareGridDiagram, start, end, came_from);

    // Then
    auto shortest_path = GetReconstructPath<GridLocation>(start, end, came_from);
    DrawGrid(kGridWithWeightsDiagram, nullptr, &came_from, &shortest_path, &start, &end);
    EXPECT_GT(shortest_path.size(), 1U);
}

}  // namespace
}  // namespace planning
