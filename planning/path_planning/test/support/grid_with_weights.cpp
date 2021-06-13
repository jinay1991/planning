///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/test/support/grid_with_weights.h"

namespace planning
{
GridWithWeights::GridWithWeights(const units::length::meter_t width, const units::length::meter_t height)
    : SquareGrid{width, height}
{
}

void GridWithWeights::AddForest(const GridLocation& location)
{
    forests_.insert(location);
}

double GridWithWeights::GetCost(const GridLocation& from, const GridLocation& to) const
{
    return ((forests_.find(to) != forests_.end()) ? 5.0 : 1.0);
}

}  // namespace planning
