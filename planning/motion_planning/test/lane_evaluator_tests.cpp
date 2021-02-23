///
/// @file
/// @brief Contains unit tests for Lane Evaluator.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/lane_evaluator.h"
#include "planning/motion_planning/roadmodel_data_source.h"
#include "planning/motion_planning/test/support/data_source_builder.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using LaneId = LaneInformation::LaneId;

class IsValidLaneIdSpec : public ::testing::TestWithParam<std::tuple<GlobalLaneId, LaneId, bool>>
{
};

TEST_P(IsValidLaneIdSpec, GivenTypicalLandId_WhenEvaluatedIsValidLaneId_ThenReturnBoolean)
{
    const auto& data_source = DataSourceBuilder().WithGlobalLaneId(std::get<0>(GetParam())).Build();
    const auto actual = LaneEvaluator(data_source).IsValidLane(std::get<1>(GetParam()));

    const auto expected = std::get<2>(GetParam());
    EXPECT_EQ(actual, expected);
}
INSTANTIATE_TEST_SUITE_P(LaneEvaluator,
                         IsValidLaneIdSpec,
                         ::testing::Values(std::make_tuple(GlobalLaneId::kCenter, LaneId::kEgo, true),
                                           std::make_tuple(GlobalLaneId::kCenter, LaneId::kLeft, true),
                                           std::make_tuple(GlobalLaneId::kCenter, LaneId::kRight, true),
                                           std::make_tuple(GlobalLaneId::kLeft, LaneId::kLeft, false),
                                           std::make_tuple(GlobalLaneId::kLeft, LaneId::kEgo, true),
                                           std::make_tuple(GlobalLaneId::kRight, LaneId::kRight, false),
                                           std::make_tuple(GlobalLaneId::kRight, LaneId::kEgo, true),
                                           std::make_tuple(GlobalLaneId::kCenter, LaneId::kInvalid, false)));

class IsDrivableSpec : public ::testing::TestWithParam<std::tuple<GlobalLaneId, GlobalLaneId, LaneId, bool>>
{
};

TEST_P(IsDrivableSpec, GivenTypicalSensorFusion_WhenCheckedIsDrivableLane_ThenReturnBasedOnCollisionAvoidance)
{
    const auto data_source = DataSourceBuilder()
                                 .WithGlobalLaneId(std::get<0>(GetParam()))
                                 .WithObjectInLane(std::get<1>(GetParam()), units::velocity::meters_per_second_t{10.0})
                                 .Build();
    const auto actual = LaneEvaluator(data_source).IsDrivableLane(std::get<2>(GetParam()));

    const auto expected = std::get<3>(GetParam());
    EXPECT_EQ(actual, expected);
}
INSTANTIATE_TEST_SUITE_P(
    LaneEvaluator,
    IsDrivableSpec,
    ::testing::Values(std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kCenter, LaneId::kEgo, false),
                      std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kCenter, LaneId::kEgo, true),
                      std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kCenter, LaneId::kEgo, true),
                      std::make_tuple(GlobalLaneId::kInvalid, GlobalLaneId::kCenter, LaneId::kEgo, false),
                      std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kLeft, LaneId::kEgo, true),
                      std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kLeft, LaneId::kEgo, true),
                      std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kLeft, LaneId::kEgo, false),
                      std::make_tuple(GlobalLaneId::kInvalid, GlobalLaneId::kLeft, LaneId::kEgo, false),
                      std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kRight, LaneId::kEgo, true),
                      std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kRight, LaneId::kEgo, false),
                      std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kRight, LaneId::kEgo, true),
                      std::make_tuple(GlobalLaneId::kInvalid, GlobalLaneId::kRight, LaneId::kEgo, false),
                      std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kInvalid, LaneId::kEgo, true),
                      std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kInvalid, LaneId::kEgo, true),
                      std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kInvalid, LaneId::kEgo, true),
                      std::make_tuple(GlobalLaneId::kInvalid, GlobalLaneId::kInvalid, LaneId::kEgo, false),
                      std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kRight, LaneId::kRight, false),
                      std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kLeft, LaneId::kLeft, false),
                      std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kLeft, LaneId::kInvalid, false)));

}  // namespace
}  // namespace planning
