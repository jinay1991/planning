///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#include "planning/path_planning/node.h"

namespace planning
{

Node::Node() : position_{}, g_cost_{0}, h_cost_{0} {}

Node::Node(const Position& position, const std::int32_t g_cost, const std::int32_t h_cost)
    : position_{position}, g_cost_{g_cost}, h_cost_{h_cost}
{
}

const Position& Node::GetPosition() const
{
    return position_;
}

std::int32_t Node::GetGCost() const
{
    return g_cost_;
}

std::int32_t Node::GetHCost() const
{
    return h_cost_;
}

std::int32_t Node::GetFCost() const
{
    return (g_cost_ + h_cost_);
}

}  // namespace planning
