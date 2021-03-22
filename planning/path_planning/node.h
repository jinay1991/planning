///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_NODE_H
#define PLANNING_PATH_PLANNING_NODE_H

#include "planning/path_planning/i_node.h"

namespace planning
{

class Node : public INode
{
  public:
    Node();
    explicit Node(const Position& position, const std::int32_t g_cost, const std::int32_t h_cost);

    const Position& GetPosition() const override;
    std::int32_t GetGCost() const override;
    std::int32_t GetHCost() const override;

    std::int32_t GetFCost() const override;

  private:
    Position position_;
    std::int32_t g_cost_;
    std::int32_t h_cost_;
};

}  // namespace planning

#endif  /// PLANNING_PATH_PLANNING_NODE_H
