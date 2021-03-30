///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/binary_heap.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{

TEST(BinaryHeap, SampleTest)
{
    // Given
    BinaryHeap<char, 7U> heap{};

    heap.Insert('a');
    heap.Insert('b');
    heap.Insert('c');
    heap.Insert('d');
    heap.Insert('e');
    heap.Insert('f');
    heap.Insert('g');

    heap.PrintHeap();
}
}  // namespace
}  // namespace planning
