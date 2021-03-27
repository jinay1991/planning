///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_DATATYPES_PATH_PLANNING_H
#define PLANNING_DATATYPES_PATH_PLANNING_H

#include <units.h>

#include <cstdint>
#include <vector>

namespace planning
{

using Node = std::uint8_t;
using Path = std::vector<Node>;

}  // namespace planning

#endif  /// PLANNING_DATATYPES_PATH_PLANNING_H
