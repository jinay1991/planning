///
/// @file lane_evaluator_test.cpp
/// @brief Contains unit tests for Lane Evaluator.
/// @copyright Copyright (c) 2020. All Rights Reserved.
///

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "motion_planning/roadmodel_data_source.h"

#include "data_source_builder.h"

#define private public
#include "motion_planning/lane_evaluator/lane_evaluator.h"

using namespace motion_planning;

namespace
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using LaneId = LaneInformation::LaneId;

class IsValidLaneIdSpec : public ::testing::TestWithParam<std::tuple<GlobalLaneId, LaneId, bool>>
{
  protected:
    virtual void SetUp() override
    {
        lane_evaluator_ =
            std::make_unique<LaneEvaluator>(DataSourceBuilder().WithGlobalLaneId(std::get<0>(GetParam())).Build());
    }

    std::unique_ptr<LaneEvaluator> lane_evaluator_;
};
TEST_P(IsValidLaneIdSpec, GivenTypicalLandId_WhenEvaluatedIsValidLaneId_ThenReturnBoolean)
{
    const auto actual = lane_evaluator_->IsValidLane(std::get<1>(GetParam()));

    const auto expected = std::get<2>(GetParam());
    EXPECT_EQ(actual, expected);
}
INSTANTIATE_TEST_SUITE_P(LaneEvaluator, IsValidLaneIdSpec,
                         ::testing::Values(std::make_tuple(GlobalLaneId::kCenter, LaneId::kEgo, true),
                                           std::make_tuple(GlobalLaneId::kCenter, LaneId::kLeft, true),
                                           std::make_tuple(GlobalLaneId::kCenter, LaneId::kRight, true),
                                           std::make_tuple(GlobalLaneId::kLeft, LaneId::kLeft, false),
                                           std::make_tuple(GlobalLaneId::kLeft, LaneId::kEgo, true),
                                           std::make_tuple(GlobalLaneId::kRight, LaneId::kRight, false),
                                           std::make_tuple(GlobalLaneId::kRight, LaneId::kEgo, true),
                                           std::make_tuple(GlobalLaneId::kCenter, LaneId::kInvalid, false)));

class GetLocalLaneSpec : public ::testing::TestWithParam<std::tuple<GlobalLaneId, GlobalLaneId, LaneId>>
{
  protected:
    virtual void SetUp() override
    {
        lane_evaluator_ = std::make_unique<LaneEvaluator>(
            DataSourceBuilder()
                .WithGlobalLaneId(std::get<0>(GetParam()))
                .WithObjectInLane(std::get<1>(GetParam()), units::velocity::meters_per_second_t{10.0})
                .Build());
    }

    std::unique_ptr<LaneEvaluator> lane_evaluator_;
};
TEST_P(GetLocalLaneSpec, GivenTypicalGlobalLaneId_WhenConverted_ThenReturnLocalLaneId)
{
    const auto actual = lane_evaluator_->GetLocalLaneId(std::get<1>(GetParam()));

    const auto expected = std::get<2>(GetParam());
    EXPECT_EQ(actual, expected);
}
INSTANTIATE_TEST_SUITE_P(LaneEvaluator, GetLocalLaneSpec,
                         ::testing::Values(std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kCenter, LaneId::kEgo),
                                           std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kLeft, LaneId::kLeft),
                                           std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kRight, LaneId::kRight),
                                           std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kCenter, LaneId::kRight),
                                           std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kLeft, LaneId::kEgo),
                                           std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kRight, LaneId::kInvalid),
                                           std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kCenter, LaneId::kLeft),
                                           std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kLeft, LaneId::kInvalid),
                                           std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kRight, LaneId::kEgo)));

class IsObjectNearSpec : public ::testing::TestWithParam<std::tuple<FrenetCoordinates, FrenetCoordinates, bool>>
{
  protected:
    virtual void SetUp() override { lane_evaluator_ = std::make_unique<LaneEvaluator>(DataSourceBuilder().Build()); }

    std::unique_ptr<LaneEvaluator> lane_evaluator_;
};
TEST_P(IsObjectNearSpec, GivenTypicalVehiclePositions_WhenEvaluatedIsObjectNear_ThenReturnBoolean)
{
    const auto actual = lane_evaluator_->IsObjectNear(std::get<0>(GetParam()), std::get<1>(GetParam()));

    const auto expected = std::get<2>(GetParam());
    EXPECT_EQ(actual, expected);
}
INSTANTIATE_TEST_SUITE_P(
    LaneEvaluator, IsObjectNearSpec,
    ::testing::Values(
        std::make_tuple(FrenetCoordinates{0, 0}, FrenetCoordinates{0, 0}, true),
        std::make_tuple(FrenetCoordinates{gkFarDistanceThreshold.value() - 15, 0}, FrenetCoordinates{0, 0}, true),
        std::make_tuple(FrenetCoordinates{gkFarDistanceThreshold.value(), 0}, FrenetCoordinates{0, 0}, false),
        std::make_tuple(FrenetCoordinates{gkFarDistanceThreshold.value() + 15, 0}, FrenetCoordinates{0, 0}, false),
        std::make_tuple(FrenetCoordinates{0, 0}, FrenetCoordinates{gkFarDistanceThreshold.value() - 15, 0}, true),
        std::make_tuple(FrenetCoordinates{0, 0}, FrenetCoordinates{gkFarDistanceThreshold.value(), 0}, false),
        std::make_tuple(FrenetCoordinates{0, 0}, FrenetCoordinates{gkFarDistanceThreshold.value() + 15, 0}, false)));

class IsDrivableSpec : public ::testing::TestWithParam<std::tuple<GlobalLaneId, GlobalLaneId, LaneId, bool>>
{
  protected:
    virtual void SetUp() override
    {
        lane_evaluator_ = std::make_unique<LaneEvaluator>(
            DataSourceBuilder()
                .WithGlobalLaneId(std::get<0>(GetParam()))
                .WithObjectInLane(std::get<1>(GetParam()), units::velocity::meters_per_second_t{10.0})
                .Build());
    }

    std::unique_ptr<LaneEvaluator> lane_evaluator_;
};

TEST_P(IsDrivableSpec, GivenTypicalSensorFusion_WhenCheckedIsDrivableLane_ThenReturnBasedOnCollisionAvoidance)
{
    const auto actual = lane_evaluator_->IsDrivableLane(std::get<2>(GetParam()));

    const auto expected = std::get<3>(GetParam());
    EXPECT_EQ(actual, expected);
}
INSTANTIATE_TEST_SUITE_P(
    LaneEvaluator, IsDrivableSpec,
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
