///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_DATATYPES_POSITION_H
#define PLANNING_DATATYPES_POSITION_H

#include <units.h>

namespace planning
{

struct Position
{
    units::length::meter_t x;
    units::length::meter_t y;
    units::length::meter_t z;
};

}  // namespace planning

#endif  /// PLANNING_DATATYPES_POSITION_H
