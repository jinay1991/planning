///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_COMMON_TREE_BINARY_TREE_H
#define PLANNING_COMMON_TREE_BINARY_TREE_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <ostream>
#include <sstream>

namespace planning
{

template <typename T, std::size_t kMaxNumberOfNodes>
class BinaryTree
{
  public:
    using value_type = T;
    using index_type = std::size_t;
    using size_type = std::size_t;
    using container = std::array<T, kMaxNumberOfNodes>;
    using reference = typename container::reference;
    using const_reference = typename container::const_reference;

    static constexpr size_type kMaxDepth = ((kMaxNumberOfNodes - 1UL) / 2UL);

    inline constexpr BinaryTree() : size_{0UL}, tree_{} {}

    inline constexpr void Insert(const value_type& value)
    {
        if (IsFull())
        {
            std::cerr << "No space left for inserting new element to the buffer!" << std::endl;
            return;
        }

        size_++;
        SetCurrentNodeValue(value);

        index_type current_node_index = GetCurrentNodeIndex();
        while ((current_node_index != 0UL) &&
               (GetNodeValue(GetParentNodeIndex(current_node_index)) < GetNodeValue(current_node_index)))
        {
            const index_type parent_node_index = GetParentNodeIndex(current_node_index);
            Swap(parent_node_index, current_node_index);
            current_node_index = parent_node_index;
        }
    }

    inline constexpr void Reset()
    {
        tree_.fill(value_type{});
        size_ = 0UL;
    }

    inline constexpr value_type GetMaxNodeValue() const { return GetNodeValue(0UL); }
    inline constexpr value_type GetNodeValue(const index_type node_index) const { return tree_.at(node_index); }

    inline constexpr size_type GetCapacity() const { return tree_.size(); }
    inline constexpr size_type GetSize() const { return size_; }
    inline constexpr size_type GetDepth() const
    {
        return static_cast<size_type>(std::ceil((static_cast<double>(GetSize()) - 1.0) / 2.0));
    }

    inline constexpr bool IsEmpty() const { return (0UL == GetSize()); }
    inline constexpr bool IsFull() const { return (GetSize() >= GetCapacity()); }

    inline constexpr std::array<value_type, kMaxDepth> GetLeftView() const
    {
        std::array<value_type, kMaxDepth> left_view{};
        index_type current_node_index = 0UL;
        index_type left_view_index = 0UL;
        left_view.at(left_view_index) = GetNodeValue(current_node_index);
        left_view_index++;
        while (current_node_index <= GetSize())
        {
            const index_type left_node_index = GetLeftNodeIndex(current_node_index);
            const value_type left_node_value = GetNodeValue(left_node_index);
            const index_type right_node_index = GetRightNodeIndex(current_node_index);
            const value_type right_node_value = GetNodeValue(right_node_index);
            if (left_node_value != 0UL)
            {
                left_view.at(left_view_index) = left_node_value;
                current_node_index = left_node_index;
            }
            else
            {
                left_view.at(left_view_index) = right_node_value;
                current_node_index = right_node_index;
            }
            left_view_index++;
        }
        return left_view;
    }

  private:
    inline constexpr index_type GetCurrentNodeIndex() const { return std::max(0UL, size_ - 1UL); }
    inline constexpr void SetCurrentNodeValue(const value_type& value) { tree_.at(GetCurrentNodeIndex()) = value; }

    inline constexpr void Swap(const index_type parent_node_index, const index_type current_node_index)
    {
        const value_type parent_node_value = GetNodeValue(parent_node_index);
        const value_type current_node_value = GetNodeValue(current_node_index);
        SetNodeValue(parent_node_index, current_node_value);
        SetNodeValue(current_node_index, parent_node_value);
    }

    inline constexpr void SetNodeValue(const index_type node_index, const value_type& value)
    {
        tree_.at(node_index) = value;
    }

    inline constexpr index_type GetLeftNodeIndex(const index_type node_index) const
    {
        return ((2UL * node_index) + 1UL);
    }

    inline constexpr index_type GetRightNodeIndex(const index_type node_index) const
    {
        return ((2UL * node_index) + 2UL);
    }

    inline constexpr index_type GetParentNodeIndex(const index_type node_index) const
    {
        if (node_index < 1UL)
        {
            return 0UL;
        }
        return ((node_index - 1UL) / 2UL);
    }
    size_type size_;
    container tree_;
};

template <typename T, std::size_t kMaxNumberOfNodes>
inline constexpr std::string to_string(const BinaryTree<T, kMaxNumberOfNodes>& tree)
{
    std::stringstream sstream;
    for (auto node_index = 0UL; node_index < tree.GetSize(); node_index++)
    {
        sstream << " " << tree.GetNodeValue(node_index);
    }
    return sstream.str();
}

template <typename T, std::size_t kMaxNumberOfNodes>
inline std::ostream& operator<<(std::ostream& stream, const BinaryTree<T, kMaxNumberOfNodes>& tree)
{
    const std::string str = to_string(tree);
    stream << str;
    return stream;
}
}  // namespace planning

#endif  /// PLANNING_COMMON_TREE_BINARY_TREE_H
