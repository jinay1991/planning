///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/astar_search_algorithm.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

namespace planning
{
namespace
{
using namespace units::literals;

class AStarSearchAlgorithmFixture : public ::testing::Test
{
  public:
    AStarSearchAlgorithmFixture() : unit_{} {}

  protected:
    void SearchShortestPath(const Position& start, const Position& end) { unit_.SearchShortestPath(start, end); }

    const ShortestPath& GetShortestPath() const { return unit_.GetShortestPath(); }

  private:
    AStarSearchAlgorithm unit_;
};

TEST_F(AStarSearchAlgorithmFixture, GivenTypicalGrid_ExpectShortestPath)
{
    // Given
    const Position start{0.0_m, 0.0_m, 0.0_m};
    const Position end{10.0_m, 0.0_m, 0.0_m};

    // When
    SearchShortestPath(start, end);

    // Then
}
}  // namespace
}  // namespace planning
