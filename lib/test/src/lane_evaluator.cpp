///
/// @file
///

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sstream>

#include <motion_planning/roadmodel_data_source.h>

#include "data_source_builder.h"

#define private public
#include <motion_planning/lane_evaluator/lane_evaluator.h>

using namespace motion_planning;

namespace
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using LaneId = LaneInformation::LaneId;

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
    // Run
    const auto actual = lane_evaluator_->GetLocalLaneId(std::get<1>(GetParam()));

    // Assert
    const auto expected = std::get<2>(GetParam());
    EXPECT_EQ(actual, expected);
}
INSTANTIATE_TEST_CASE_P(LaneEvaluator, GetLocalLaneSpec,
                        ::testing::Values(std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kCenter, LaneId::kEgo),
                                          std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kLeft, LaneId::kLeft),
                                          std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kRight, LaneId::kRight),
                                          std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kCenter, LaneId::kRight),
                                          std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kLeft, LaneId::kEgo),
                                          std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kRight, LaneId::kInvalid),
                                          std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kCenter, LaneId::kLeft),
                                          std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kLeft, LaneId::kInvalid),
                                          std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kRight, LaneId::kEgo)));

class GetGlobalLaneSpec : public ::testing::TestWithParam<std::tuple<FrenetCoordinates, GlobalLaneId>>
{
  protected:
    virtual void SetUp() override { lane_evaluator_ = std::make_unique<LaneEvaluator>(DataSourceBuilder().Build()); }

    std::unique_ptr<LaneEvaluator> lane_evaluator_;
};
TEST_P(GetGlobalLaneSpec, GivenTypicalFrenetCoordinates_WhenGetGlobalLaneId_ThenReturnGlobalLaneId)
{
    // Run
    const auto actual = lane_evaluator_->GetGlobalLaneId(std::get<0>(GetParam()));

    // Assert
    EXPECT_EQ(actual, std::get<1>(GetParam()));
}
INSTANTIATE_TEST_CASE_P(LaneEvaluator, GetGlobalLaneSpec,
                        ::testing::Values(std::make_tuple(FrenetCoordinates{0, 0}, GlobalLaneId::kInvalid),
                                          std::make_tuple(FrenetCoordinates{0, 2}, GlobalLaneId::kLeft),
                                          std::make_tuple(FrenetCoordinates{0, 4}, GlobalLaneId::kInvalid),
                                          std::make_tuple(FrenetCoordinates{0, 6}, GlobalLaneId::kCenter),
                                          std::make_tuple(FrenetCoordinates{0, 8}, GlobalLaneId::kInvalid),
                                          std::make_tuple(FrenetCoordinates{0, 10}, GlobalLaneId::kRight),
                                          std::make_tuple(FrenetCoordinates{0, 12}, GlobalLaneId::kInvalid)));

class IsObjectNearSpec : public ::testing::TestWithParam<std::tuple<FrenetCoordinates, FrenetCoordinates, bool>>
{
  protected:
    virtual void SetUp() override { lane_evaluator_ = std::make_unique<LaneEvaluator>(DataSourceBuilder().Build()); }

    std::unique_ptr<LaneEvaluator> lane_evaluator_;
};
TEST_P(IsObjectNearSpec, GivenTypicalVehiclePositions_WhenEvaluatedIsObjectNear_ThenReturnBoolean)
{
    // Run
    const auto actual = lane_evaluator_->IsObjectNear(std::get<0>(GetParam()), std::get<1>(GetParam()));

    // Assert
    const auto expected = std::get<2>(GetParam());
    EXPECT_EQ(actual, expected);
}
INSTANTIATE_TEST_CASE_P(
    LaneEvaluator, IsObjectNearSpec,
    ::testing::Values(
        std::make_tuple(FrenetCoordinates{0, 0}, FrenetCoordinates{0, 0}, true),
        std::make_tuple(FrenetCoordinates{gkFarDistanceThreshold.value() - 15, 0}, FrenetCoordinates{0, 0}, true),
        std::make_tuple(FrenetCoordinates{gkFarDistanceThreshold.value(), 0}, FrenetCoordinates{0, 0}, false),
        std::make_tuple(FrenetCoordinates{gkFarDistanceThreshold.value() + 15, 0}, FrenetCoordinates{0, 0}, false),
        std::make_tuple(FrenetCoordinates{0, 0}, FrenetCoordinates{gkFarDistanceThreshold.value() - 15, 0}, true),
        std::make_tuple(FrenetCoordinates{0, 0}, FrenetCoordinates{gkFarDistanceThreshold.value(), 0}, false),
        std::make_tuple(FrenetCoordinates{0, 0}, FrenetCoordinates{gkFarDistanceThreshold.value() + 15, 0}, false)));

class IsDrivableSpec : public ::testing::TestWithParam<std::tuple<GlobalLaneId, GlobalLaneId, bool>>
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
    // Run
    const auto actual = lane_evaluator_->IsDrivableLane(LaneId::kEgo);

    // Assert
    const auto expected = std::get<2>(GetParam());
    EXPECT_EQ(actual, expected);
}
INSTANTIATE_TEST_CASE_P(LaneEvaluator, IsDrivableSpec,
                        ::testing::Values(std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kCenter, false),
                                          std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kCenter, true),
                                          std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kCenter, true),
                                          std::make_tuple(GlobalLaneId::kInvalid, GlobalLaneId::kCenter, false),
                                          std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kLeft, true),
                                          std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kLeft, true),
                                          std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kLeft, false),
                                          std::make_tuple(GlobalLaneId::kInvalid, GlobalLaneId::kLeft, false),
                                          std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kRight, true),
                                          std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kRight, false),
                                          std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kRight, true),
                                          std::make_tuple(GlobalLaneId::kInvalid, GlobalLaneId::kRight, false),
                                          std::make_tuple(GlobalLaneId::kCenter, GlobalLaneId::kInvalid, true),
                                          std::make_tuple(GlobalLaneId::kRight, GlobalLaneId::kInvalid, true),
                                          std::make_tuple(GlobalLaneId::kLeft, GlobalLaneId::kInvalid, true),
                                          std::make_tuple(GlobalLaneId::kInvalid, GlobalLaneId::kInvalid, false)));

}  // namespace
