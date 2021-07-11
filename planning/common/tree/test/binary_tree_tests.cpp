///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/common/tree/binary_tree.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <ostream>
#include <sstream>

namespace planning
{
namespace
{
constexpr std::size_t kMaxNumberOfNodes{10UL};

class BinaryTreeFixture : public ::testing::Test
{
  public:
    BinaryTreeFixture() : unit_{} {}

  protected:
    BinaryTree<std::int32_t, kMaxNumberOfNodes> unit_;
};

TEST_F(BinaryTreeFixture, Insert_GivenTypicalNode_ExpectSameNodeInserted)
{
    // Given
    const std::int32_t node = 3;
    ASSERT_EQ(unit_.GetSize(), 0UL);

    // When
    unit_.Insert(node);

    // Then
    EXPECT_EQ(unit_.GetSize(), 1UL);
    EXPECT_EQ(unit_.GetNodeValue(0UL), node);
}

TEST_F(BinaryTreeFixture, Insert_GivenTypicalNodes_ExpectSameNodesInserted)
{
    // Given
    ASSERT_EQ(unit_.GetSize(), 0UL);

    // When
    unit_.Insert(3);  // root
    unit_.Insert(2);  // left
    unit_.Insert(1);  // left

    // Then
    EXPECT_EQ(unit_.GetSize(), 3UL);
    EXPECT_EQ(unit_.GetNodeValue(0UL), 3);
    EXPECT_EQ(unit_.GetNodeValue(1UL), 2);
    EXPECT_EQ(unit_.GetNodeValue(2UL), 1);
}

TEST_F(BinaryTreeFixture, Reset_GivenTypicalBinaryTree_ExpectResetValues)
{
    // Given
    unit_.Insert(3);
    unit_.Insert(2);
    unit_.Insert(4);
    ASSERT_EQ(unit_.GetSize(), 3UL);

    // When
    unit_.Reset();

    // Then
    EXPECT_EQ(unit_.GetSize(), 0UL);
}

TEST_F(BinaryTreeFixture, IsEmpty_GivenTypicalBinaryTree_ExpectTrue)
{
    // Given
    ASSERT_EQ(unit_.GetSize(), 0UL);

    // When/Then
    EXPECT_TRUE(unit_.IsEmpty());
}

TEST_F(BinaryTreeFixture, IsEmpty_GivenTypicalBinaryTree_ExpectFalse)
{
    // Given
    unit_.Insert(10);
    ASSERT_EQ(unit_.GetSize(), 1UL);

    // When/Then
    EXPECT_FALSE(unit_.IsEmpty());
}

TEST_F(BinaryTreeFixture, IsFull_GivenTypicalBinaryTree_ExpectTrue)
{
    // Given
    unit_.Insert(10);
    unit_.Insert(9);
    unit_.Insert(8);
    unit_.Insert(7);
    unit_.Insert(6);
    unit_.Insert(5);
    unit_.Insert(4);
    unit_.Insert(3);
    unit_.Insert(2);
    unit_.Insert(1);
    ASSERT_EQ(unit_.GetSize(), kMaxNumberOfNodes);

    // When/Then
    EXPECT_TRUE(unit_.IsFull());
}

TEST_F(BinaryTreeFixture, IsFull_GivenTypicalBinaryTree_ExpectFalse)
{
    // Given
    unit_.Insert(10);
    unit_.Insert(9);
    unit_.Insert(8);
    unit_.Insert(7);
    unit_.Insert(6);
    unit_.Insert(5);
    unit_.Insert(4);
    unit_.Insert(3);
    unit_.Insert(2);
    ASSERT_EQ(unit_.GetSize(), 9UL);

    // When/Then
    EXPECT_FALSE(unit_.IsFull());
}

TEST_F(BinaryTreeFixture, StreamOperator_GivenTypicalBinaryTree_ExpectStreamRepresentationOfTree)
{
    // Given
    std::stringstream stream;
    unit_.Insert(3);
    unit_.Insert(2);
    unit_.Insert(1);
    unit_.Insert(15);
    unit_.Insert(5);
    unit_.Insert(4);
    unit_.Insert(45);
    ASSERT_EQ(unit_.GetSize(), 7UL);

    // When/Then
    stream << unit_;

    // Then
    EXPECT_EQ(stream.str(), " 45 5 15 2 3 1 4");
}

TEST_F(BinaryTreeFixture, GetMaxNodeValue_GivenTypicalBinaryTree_ExpectMaxNodeValue)
{
    // Given
    unit_.Insert(3);
    unit_.Insert(2);
    unit_.Insert(1);
    unit_.Insert(15);
    unit_.Insert(5);
    unit_.Insert(4);
    unit_.Insert(45);
    ASSERT_EQ(unit_.GetSize(), 7UL);

    // When/Then
    const auto max_value = unit_.GetMaxNodeValue();

    // Then
    EXPECT_EQ(max_value, 45);
}

TEST_F(BinaryTreeFixture, GetDepth_GivenTypicalBinaryTree_ExpectBinaryTreeDepth)
{
    // Given
    unit_.Insert(3);
    unit_.Insert(2);
    unit_.Insert(1);
    unit_.Insert(15);
    unit_.Insert(5);
    unit_.Insert(45);
    ASSERT_EQ(unit_.GetSize(), 6UL);

    // When/Then
    const auto depth = unit_.GetDepth();

    // Then
    EXPECT_EQ(depth, 3UL);
}

TEST_F(BinaryTreeFixture, GetLeftView_GivenTypicalBinaryTree_ExpectLeftViewOfBinaryTree)
{
    // Given
    unit_.Insert(3);
    unit_.Insert(2);
    unit_.Insert(1);
    unit_.Insert(15);
    unit_.Insert(5);
    unit_.Insert(4);
    unit_.Insert(45);
    ASSERT_EQ(unit_.GetSize(), 7UL);

    // When/Then
    const auto left_view = unit_.GetLeftView();

    // Then
    ASSERT_EQ(left_view.size(), 4UL);
    EXPECT_EQ(left_view.at(0UL), 45);
    EXPECT_EQ(left_view.at(1UL), 5);
    EXPECT_EQ(left_view.at(2UL), 2);
    EXPECT_EQ(left_view.at(3UL), 0);  // empty nodes
}

}  // namespace
}  // namespace planning
