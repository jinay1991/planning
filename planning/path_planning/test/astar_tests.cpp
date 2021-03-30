///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/astar.h"
#include "planning/path_planning/dijkstra.h"
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

// TEST(SquareGrid, AddBlock)
// {
//     // Given
//     SquareGrid grid{30_m, 15_m};
//     GridLocation start{3_m, 3_m};
//     GridLocation end{5_m, 12_m};

//     // When
//     grid.AddBlock(start, end);

//     // Then
//     DrawGrid(grid);
// }

TEST(AStar, SearchShortestPath_GivenTypicalStartEndLocations_ExpectShortestPath)
{
    // Given
    const GridLocation start{0_m, 0_m};
    const GridLocation end{8_m, 5_m};
    AStar<GridWithWeights, GridLocation> unit_{kGridWithWeightsDiagram};

    // When
    unit_.SearchShortestPath(start, end);

    // Then
    EXPECT_THAT(unit_.GetShortestPath(),
                ::testing::ElementsAre(start,
                                       GridLocation{1_m, 0_m},
                                       GridLocation{2_m, 0_m},
                                       GridLocation{3_m, 0_m},
                                       GridLocation{4_m, 0_m},
                                       GridLocation{5_m, 0_m},
                                       GridLocation{6_m, 0_m},
                                       GridLocation{6_m, 1_m},
                                       GridLocation{7_m, 1_m},
                                       GridLocation{7_m, 2_m},
                                       GridLocation{8_m, 2_m},
                                       GridLocation{8_m, 3_m},
                                       GridLocation{8_m, 4_m},
                                       end));
}

TEST(Dijkstra, SearchShortestPath_GivenTypicalStartEndLocations_ExpectShortestPath)
{
    // Given
    const GridLocation start{0_m, 0_m};
    const GridLocation end{8_m, 5_m};
    Dijkstra<GridWithWeights, GridLocation> unit_{kGridWithWeightsDiagram};

    // When
    unit_.SearchShortestPath(start, end);

    // Then
    EXPECT_THAT(unit_.GetShortestPath(),
                ::testing::ElementsAre(start,
                                       GridLocation{1_m, 0_m},
                                       GridLocation{2_m, 0_m},
                                       GridLocation{3_m, 0_m},
                                       GridLocation{4_m, 0_m},
                                       GridLocation{5_m, 0_m},
                                       GridLocation{6_m, 0_m},
                                       GridLocation{7_m, 0_m},
                                       GridLocation{7_m, 1_m},
                                       GridLocation{7_m, 2_m},
                                       GridLocation{8_m, 2_m},
                                       GridLocation{8_m, 3_m},
                                       GridLocation{8_m, 4_m},
                                       end));
}

// TEST(BreadthFirstSearch, GivenTypicalGridDiagram_ExpectShortestPath)
// {
//     // Given
//     std::unordered_map<GridLocation, GridLocation> came_from;
//     GridLocation start{0_m, 0_m};
//     GridLocation end{8_m, 5_m};

//     // When
//     BreadthFirstSearch(kSquareGridDiagram, start, end, came_from);

//     // Then
//     auto shortest_path = GetReconstructPath<GridLocation>(start, end, came_from);
//     DrawGrid(kGridWithWeightsDiagram, nullptr, &came_from, &shortest_path, &start, &end);
//     EXPECT_GT(shortest_path.size(), 1U);
// }

}  // namespace
}  // namespace planning
