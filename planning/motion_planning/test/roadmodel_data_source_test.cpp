///
/// @file roadmodel_data_source_test.cpp
/// @brief Contains unit tests for Road Model Data Source.
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include "planning/motion_planning/roadmodel_data_source.h"
#include "planning/motion_planning/i_data_source.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
class GetGlobalLaneSpec : public ::testing::TestWithParam<std::tuple<FrenetCoordinates, GlobalLaneId>>
{
  protected:
    virtual void SetUp() override { data_source_ = std::make_unique<RoadModelDataSource>(); }

    std::unique_ptr<IDataSource> data_source_;
};

TEST_P(GetGlobalLaneSpec, GivenTypicalFrenetCoordinates_WhenGetGlobalLaneId_ThenReturnGlobalLaneId)
{
    const auto actual = data_source_->GetGlobalLaneId(std::get<0>(GetParam()));

    EXPECT_EQ(actual, std::get<1>(GetParam()));
}
INSTANTIATE_TEST_SUITE_P(DataSourceSpec, GetGlobalLaneSpec,
                         ::testing::Values(std::make_tuple(FrenetCoordinates{0, 0}, GlobalLaneId::kInvalid),
                                           std::make_tuple(FrenetCoordinates{0, 2}, GlobalLaneId::kLeft),
                                           std::make_tuple(FrenetCoordinates{0, 4}, GlobalLaneId::kInvalid),
                                           std::make_tuple(FrenetCoordinates{0, 6}, GlobalLaneId::kCenter),
                                           std::make_tuple(FrenetCoordinates{0, 8}, GlobalLaneId::kInvalid),
                                           std::make_tuple(FrenetCoordinates{0, 10}, GlobalLaneId::kRight),
                                           std::make_tuple(FrenetCoordinates{0, 12}, GlobalLaneId::kInvalid)));
}  // namespace planning