///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_I_NODE_H
#define PLANNING_PATH_PLANNING_I_NODE_H

#include "planning/datatypes/position.h"

#include <cstdint>

namespace planning
{
class INode
{
  public:
    virtual ~INode() = default;

    virtual const Position& GetPosition() const = 0;
    virtual std::int32_t GetHCost() const = 0;
    virtual std::int32_t GetGCost() const = 0;
    virtual std::int32_t GetFCost() const = 0;
};

}  // namespace planning

#endif  /// PLANNING_PATH_PLANNING_I_NODE_H
