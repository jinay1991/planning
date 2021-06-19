///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#include "planning/path_planning/test/support/square_grid_builder.h"

namespace planning
{

SquareGridBuilder::SquareGridBuilder(const units::length::meter_t width, const units::length::meter_t height)
    : grid_{width, height}
{
}

SquareGridBuilder& SquareGridBuilder::WithBlock(const GridLocation& start, const GridLocation& end)
{
    grid_.AddBlock(start, end);
    return *this;
}

const SquareGrId SquareGridBuilder::Build() const
{
    return grid_;
}

}  // namespace planning
