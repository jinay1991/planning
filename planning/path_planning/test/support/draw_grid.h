///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_TEST_SUPPORT_DRAW_GRID_H
#define PLANNING_PATH_PLANNING_TEST_SUPPORT_DRAW_GRID_H

#include "planning/path_planning/test/support/astar.h"

#include <units.h>

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace planning
{

using namespace units::literals;

// This outputs a grid. Pass in a distances map if you want to print
// the distances, or pass in a point_to map if you want to print
// arrows that point to the parent location, or pass in a path vector
// if you want to draw the path.
template <class Graph>
void DrawGrid(const Graph& graph,
              std::unordered_map<GridLocation, double>* distances = nullptr,
              std::unordered_map<GridLocation, GridLocation>* point_to = nullptr,
              std::vector<GridLocation>* path = nullptr,
              GridLocation* start = nullptr,
              GridLocation* goal = nullptr)
{
    const std::int32_t field_width = 3;
    const std::int32_t width = static_cast<std::int32_t>(graph.GetWidth().value());
    std::cout << std::string(field_width * width, '_') << '\n';
    for (auto y = 0_m; y != graph.GetHeight(); ++y)
    {
        for (auto x = 0_m; x != graph.GetWidth(); ++x)
        {
            GridLocation id{x, y};
            if (graph.IsBlocked(id))
            {
                std::cout << std::string(field_width, '#');
            }
            else if (start && id == *start)
            {
                std::cout << " A ";
            }
            else if (goal && id == *goal)
            {
                std::cout << " Z ";
            }
            else if (path != nullptr && find(path->begin(), path->end(), id) != path->end())
            {
                std::cout << " @ ";
            }
            else if (point_to != nullptr && point_to->count(id))
            {
                GridLocation next = (*point_to)[id];
                if (next.x == x + 1_m)
                {
                    std::cout << " > ";
                }
                else if (next.x == x - 1_m)
                {
                    std::cout << " < ";
                }
                else if (next.y == y + 1_m)
                {
                    std::cout << " v ";
                }
                else if (next.y == y - 1_m)
                {
                    std::cout << " ^ ";
                }
                else
                {
                    std::cout << " * ";
                }
            }
            else if (distances != nullptr && distances->count(id))
            {
                std::cout << ' ' << std::left << std::setw(field_width - 1) << (*distances)[id];
            }
            else
            {
                std::cout << " . ";
            }
        }
        std::cout << '\n';
    }
    std::cout << std::string(field_width * width, '~') << '\n';
}
}  // namespace planning

#endif  /// PLANNING_PATH_PLANNING_TEST_SUPPORT_DRAW_GRID_H
