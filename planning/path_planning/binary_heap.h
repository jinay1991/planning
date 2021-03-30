///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_BINARY_HEAP_H
#define PLANNING_PATH_PLANNING_BINARY_HEAP_H

#include <algorithm>
#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <utility>

namespace planning
{

template <typename T, std::size_t kMaxHeapSize>
class BinaryHeap
{
  public:
    using ValueType = T;
    using ContainerType = std::array<T, kMaxHeapSize>;

    constexpr BinaryHeap() : size_{}, buffer_{} {}

    constexpr std::size_t GetCapacity() const { return buffer_.size(); }
    constexpr std::size_t GetSize() const { return size_; }
    constexpr bool IsEmpty() const { return (0U == GetSize()); }
    constexpr bool IsFull() const { return (GetCapacity() == GetSize()); }

    constexpr void Insert(const T& value)
    {
        if (IsFull())
        {
            std::cerr << "Heap is full!! Can not add more values...\n";
            return;
        }

        buffer_.at(size_) = value;
        size_++;

        UpdateHeap();
    }

    constexpr void PrintHeap() const noexcept
    {
        std::cout << "Heap: [";
        for (auto i = 0U; i < GetSize(); ++i)
        {
            std::cout << ((i == 0U) ? "" : " ") << buffer_.at(i);
        }
        std::cout << "]";
        std::cout << std::endl;
    }

  private:
    constexpr void UpdateHeap()
    {
        std::size_t i = size_ - 1U;
        while (i != 0 && buffer_.at(GetParentIndex(i)) > buffer_.at(i))
        {
            const auto temp = buffer_.at(i);
            buffer_.at(i) = buffer_.at(GetParentIndex(i));
            buffer_.at(GetParentIndex(i)) = temp;
            i = GetParentIndex(i);
        }
    }

    constexpr std::size_t GetParentIndex(const std::size_t current_index) const { return (current_index) / 2U; }
    constexpr std::size_t GetLeftIndex(const std::size_t current_index) const { return (current_index * 2U); }
    constexpr std::size_t GetRightIndex(const std::size_t current_index) const { return (current_index * 2U) + 1U; }

    std::size_t size_;
    ContainerType buffer_;
};

}  // namespace planning

#endif  /// PLANNING_PATH_PLANNING_BINARY_HEAP_H
