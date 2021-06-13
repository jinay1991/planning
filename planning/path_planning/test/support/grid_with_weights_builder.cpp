///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/test/support/grid_with_weights_builder.h"

namespace planning
{

GridWithWeightsBuilder::GridWithWeightsBuilder(const units::length::meter_t width, const units::length::meter_t height)
    : grid_{width, height}
{
}

GridWithWeightsBuilder& GridWithWeightsBuilder::WithForest(const GridLocation& location)
{
    grid_.AddForest(location);
    return *this;
}

GridWithWeightsBuilder& GridWithWeightsBuilder::WithForestList(const std::initializer_list<GridLocation>& location_list)
{
    for (auto& location : location_list)
    {
        grid_.AddForest(location);
    }
    return *this;
}

GridWithWeightsBuilder& GridWithWeightsBuilder::WithBlock(const GridLocation& start, const GridLocation& end)
{
    grid_.AddBlock(start, end);
    return *this;
}

const GridWithWeights& GridWithWeightsBuilder::Build() const
{
    return grid_;
}

}  // namespace planning
