///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/graph.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{

TEST(Graph, AddEdge_GivenTypicalInputs_ExpectUpdatedEdge)
{
    // Given
    static constexpr std::size_t kMaxVertices = 4U;
    Graph<kMaxVertices> graph{};
    ASSERT_THAT(graph.GetAdjacentVertices('a'), ::testing::IsEmpty());

    // When
    graph.AddEdge('a', 'b');

    // Then
    EXPECT_THAT(graph.GetAdjacentVertices('a'), ::testing::ElementsAre('a', 'b'));
    EXPECT_THAT(graph.GetAdjacentVertices('b'), ::testing::IsEmpty());
}

TEST(Graph, AddEdge)
{
    // Given
    static constexpr std::size_t kMaxVertices = 4U;
    Graph<kMaxVertices> graph{};
    ASSERT_THAT(graph.GetAdjacentVertices('a'), ::testing::IsEmpty());

    // When
    graph.AddEdge('a', 'b');
    graph.AddEdge('a', 'c');

    // Then
    EXPECT_THAT(graph.GetAdjacentVertices('a'), ::testing::ElementsAre('a', 'b', 'c'));
    EXPECT_THAT(graph.GetAdjacentVertices('b'), ::testing::IsEmpty());
}

}  // namespace
}  // namespace planning
