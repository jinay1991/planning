///
/// @file velocity_planner_test.cpp
/// @brief Contains unit tests for Velocity Planner.
/// @copyright Copyright (c) 2020. All Rights Reserved.
///

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "motion_planning/domain_model/lane.h"
#include "motion_planning/velocity_planner/velocity_planner.h"

#include "data_source_builder.h"

using namespace motion_planning;

namespace
{
using LaneId = LaneInformation::LaneId;
using GlobalLaneId = LaneInformation::GlobalLaneId;

class VelocityPlannerSpec
{
  public:
    virtual void Init(const units::velocity::meters_per_second_t& obj_velocity, const GlobalLaneId& obj_lane_id)
    {
        data_source_ = DataSourceBuilder()
                           .WithVelocity(velocity_)
                           .WithGlobalLaneId(GlobalLaneId::kCenter)
                           .WithDistance(units::length::meter_t{0.0})
                           .WithObjectInLane(obj_lane_id, obj_velocity)
                           .Build();
        unit_ = std::make_unique<VelocityPlanner>(data_source_, velocity_);

        unit_->CalculateTargetVelocity();
    }

  protected:
    const units::velocity::meters_per_second_t velocity_{17.0};
    std::shared_ptr<IDataSource> data_source_;
    std::unique_ptr<VelocityPlanner> unit_;
};

class AccelerationSpecFixture
    : public VelocityPlannerSpec,
      public ::testing::TestWithParam<std::tuple<units::velocity::meters_per_second_t, GlobalLaneId>>
{
  protected:
    virtual void SetUp() override { VelocityPlannerSpec::Init(std::get<0>(GetParam()), std::get<1>(GetParam())); }
};

TEST_P(AccelerationSpecFixture, GivenNoClosestInPathVehicle_WhenCalculateTargetVelocity_ThenAcceleratedTargetVelocity)
{
    const auto actual = unit_->GetTargetVelocity();

    EXPECT_GT(actual, velocity_);
    EXPECT_LT(actual, data_source_->GetSpeedLimit());
}

INSTANTIATE_TEST_SUITE_P(VelocityPlanner, AccelerationSpecFixture,
                         ::testing::Combine(::testing::Values(units::velocity::meters_per_second_t{0.0},
                                                              units::velocity::meters_per_second_t{10.0},
                                                              units::velocity::meters_per_second_t{17.0},
                                                              units::velocity::meters_per_second_t{20.0}),
                                            ::testing::Values(GlobalLaneId::kLeft, GlobalLaneId::kRight)));

class DecelerationSpecFixture
    : public VelocityPlannerSpec,
      public ::testing::TestWithParam<std::tuple<units::velocity::meters_per_second_t, GlobalLaneId>>
{
  protected:
    virtual void SetUp() override { VelocityPlannerSpec::Init(std::get<0>(GetParam()), std::get<1>(GetParam())); }
};

TEST_P(DecelerationSpecFixture, GivenClosestInPathVehicle_WhenCalculateTargetVelocity_ThenDeceleratedTargetVelocity)
{
    const auto actual = unit_->GetTargetVelocity();

    EXPECT_LT(actual, velocity_);
    EXPECT_LT(actual, data_source_->GetSpeedLimit());
}

INSTANTIATE_TEST_SUITE_P(VelocityPlanner, DecelerationSpecFixture,
                         ::testing::Combine(::testing::Values(units::velocity::meters_per_second_t{0.0},
                                                              units::velocity::meters_per_second_t{15.0},
                                                              units::velocity::meters_per_second_t{17.0}),
                                            ::testing::Values(GlobalLaneId::kCenter)));

}  // namespace