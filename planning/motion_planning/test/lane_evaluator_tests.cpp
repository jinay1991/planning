///
/// @file
/// @brief Contains unit tests for Lane Evaluator.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/data_source.h"
#include "planning/motion_planning/lane_evaluator.h"
#include "planning/motion_planning/test/support/builders/data_source_builder.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{
using GlobalLaneId = LaneInformation::GlobalLaneId;
using LaneId = LaneInformation::LaneId;

template <typename T>
class LaneEvaluatorFixtureT : public ::testing::TestWithParam<T>
{
};

struct TestValidLaneIdParam
{
    // Given
    GlobalLaneId ego_global_lane_id;
    LaneId lane_id;

    // Then
    bool is_valid;
};

using LaneEvaluatorFixture_WithValidLaneId = LaneEvaluatorFixtureT<TestValidLaneIdParam>;

// clang-format off
INSTANTIATE_TEST_SUITE_P(
    LaneEvaluator,
    LaneEvaluatorFixture_WithValidLaneId,
    ::testing::Values(
        //                   ego_global_lane_id   , lane_id         , (expected) is_valid
        TestValidLaneIdParam{GlobalLaneId::kCenter, LaneId::kEgo    ,             true},
        TestValidLaneIdParam{GlobalLaneId::kCenter, LaneId::kLeft   ,             true},
        TestValidLaneIdParam{GlobalLaneId::kCenter, LaneId::kRight  ,             true},
        TestValidLaneIdParam{GlobalLaneId::kLeft  , LaneId::kLeft   ,            false},
        TestValidLaneIdParam{GlobalLaneId::kLeft  , LaneId::kEgo    ,             true},
        TestValidLaneIdParam{GlobalLaneId::kRight , LaneId::kRight  ,            false},
        TestValidLaneIdParam{GlobalLaneId::kRight , LaneId::kEgo    ,             true},
        TestValidLaneIdParam{GlobalLaneId::kCenter, LaneId::kInvalid,            false}));
// clang-format on

TEST_P(LaneEvaluatorFixture_WithValidLaneId, IsValidLane_GivenTypicalLandId_ExpectValidity)
{
    // Given
    const auto param = GetParam();
    const auto data_source = DataSourceBuilder().WithGlobalLaneId(param.ego_global_lane_id).Build();

    // When
    const auto is_valid = LaneEvaluator(data_source).IsValidLane(param.lane_id);

    // Then
    EXPECT_EQ(is_valid, param.is_valid);
}

struct TestDrivableLaneIdParam
{
    // Given
    GlobalLaneId ego_global_lane_id;
    GlobalLaneId object_global_lane_id;
    LaneId lane_id;

    // Then
    bool is_drivable;
};

using LaneEvaluatorFixture_WithDrivableLaneId = LaneEvaluatorFixtureT<TestDrivableLaneIdParam>;

// clang-format off
INSTANTIATE_TEST_SUITE_P(
    LaneEvaluator,
    LaneEvaluatorFixture_WithDrivableLaneId,
    ::testing::Values(
        //                      ego_global_lane_id    , object_global_lane_id , lane_id         , (expected) is_drivable?
        TestDrivableLaneIdParam{GlobalLaneId::kCenter , GlobalLaneId::kCenter , LaneId::kEgo    ,               false},
        TestDrivableLaneIdParam{GlobalLaneId::kRight  , GlobalLaneId::kCenter , LaneId::kEgo    ,                true},
        TestDrivableLaneIdParam{GlobalLaneId::kLeft   , GlobalLaneId::kCenter , LaneId::kEgo    ,                true},
        TestDrivableLaneIdParam{GlobalLaneId::kInvalid, GlobalLaneId::kCenter , LaneId::kEgo    ,               false},
        TestDrivableLaneIdParam{GlobalLaneId::kCenter , GlobalLaneId::kLeft   , LaneId::kEgo    ,                true},
        TestDrivableLaneIdParam{GlobalLaneId::kRight  , GlobalLaneId::kLeft   , LaneId::kEgo    ,                true},
        TestDrivableLaneIdParam{GlobalLaneId::kLeft   , GlobalLaneId::kLeft   , LaneId::kEgo    ,               false},
        TestDrivableLaneIdParam{GlobalLaneId::kInvalid, GlobalLaneId::kLeft   , LaneId::kEgo    ,               false},
        TestDrivableLaneIdParam{GlobalLaneId::kCenter , GlobalLaneId::kRight  , LaneId::kEgo    ,                true},
        TestDrivableLaneIdParam{GlobalLaneId::kRight  , GlobalLaneId::kRight  , LaneId::kEgo    ,               false},
        TestDrivableLaneIdParam{GlobalLaneId::kLeft   , GlobalLaneId::kRight  , LaneId::kEgo    ,                true},
        TestDrivableLaneIdParam{GlobalLaneId::kInvalid, GlobalLaneId::kRight  , LaneId::kEgo    ,               false},
        TestDrivableLaneIdParam{GlobalLaneId::kCenter , GlobalLaneId::kInvalid, LaneId::kEgo    ,                true},
        TestDrivableLaneIdParam{GlobalLaneId::kRight  , GlobalLaneId::kInvalid, LaneId::kEgo    ,                true},
        TestDrivableLaneIdParam{GlobalLaneId::kLeft   , GlobalLaneId::kInvalid, LaneId::kEgo    ,                true},
        TestDrivableLaneIdParam{GlobalLaneId::kInvalid, GlobalLaneId::kInvalid, LaneId::kEgo    ,               false},
        TestDrivableLaneIdParam{GlobalLaneId::kCenter , GlobalLaneId::kRight  , LaneId::kRight  ,               false},
        TestDrivableLaneIdParam{GlobalLaneId::kCenter , GlobalLaneId::kLeft   , LaneId::kLeft   ,               false},
        TestDrivableLaneIdParam{GlobalLaneId::kCenter , GlobalLaneId::kLeft   , LaneId::kInvalid,               false}
));
// clang-format on

TEST_P(LaneEvaluatorFixture_WithDrivableLaneId,
       IsDrivableLane_GivenTypicalDataSource_ExpectDrivabilityBasedOnCollisionAvoidance)
{
    // Given
    const auto param = GetParam();
    const auto data_source =
        DataSourceBuilder()
            .WithGlobalLaneId(param.ego_global_lane_id)
            .WithObjectInLane(param.object_global_lane_id, units::velocity::meters_per_second_t{10.0})
            .Build();

    // When
    const auto is_drivable = LaneEvaluator(data_source).IsDrivableLane(param.lane_id);

    // Then
    EXPECT_EQ(is_drivable, param.is_drivable);
}

}  // namespace
}  // namespace planning
