///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/test/support/simple_graph.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{

TEST(SimpleGraph, AddEdge_GivenTypicalInputs_ExpectUpdatedEdge)
{
    // Given
    SimpleGraph<char> graph{};
    ASSERT_THAT(graph.GetNeighbors('a'), ::testing::IsEmpty());

    // When
    graph.AddEdge('a', {'b'});

    // Then
    EXPECT_THAT(graph.GetNeighbors('a'), ::testing::ElementsAre('a', 'b'));
    EXPECT_THAT(graph.GetNeighbors('b'), ::testing::IsEmpty());
}

TEST(SimpleGraph, AddEdge)
{
    // Given
    SimpleGraph<char> graph{};
    ASSERT_THAT(graph.GetNeighbors('a'), ::testing::IsEmpty());

    // When
    graph.AddEdge('a', {'b'});
    graph.AddEdge('a', {'c'});

    // Then
    EXPECT_THAT(graph.GetNeighbors('a'), ::testing::ElementsAre('a', 'b'));
    EXPECT_THAT(graph.GetNeighbors('b'), ::testing::IsEmpty());
}

}  // namespace
}  // namespace planning
