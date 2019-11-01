///
/// @file
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
        unit_ = std::make_unique<VelocityPlanner>(DataSourceBuilder()
                                                      .WithVelocity(units::velocity::meters_per_second_t{17.0})
                                                      .WithGlobalLaneId(GlobalLaneId::kCenter)
                                                      .WithObjectInLane(obj_lane_id, obj_velocity)
                                                      .Build());

        unit_->CalculateTargetVelocity();
    }

  protected:
    const units::velocity::meters_per_second_t ego_accelerated_velocity_{17.2};
    const units::velocity::meters_per_second_t ego_decelerated_velocity_{16.8};

    std::unique_ptr<VelocityPlanner> unit_;
};

class AccelerationSpecFixture
    : public VelocityPlannerSpec,
      public ::testing::TestWithParam<std::tuple<units::velocity::meters_per_second_t, GlobalLaneId>>
{
  protected:
    virtual void SetUp() override { VelocityPlannerSpec::Init(std::get<0>(GetParam()), std::get<1>(GetParam())); }
};

TEST_P(AccelerationSpecFixture,
       DISABLED_GivenNoClosestInPathVehicle_WhenCalculateTargetVelocity_ThenAcceleratedTargetVelocity)
{
    const auto actual = unit_->GetTargetVelocity();

    EXPECT_EQ(actual, ego_accelerated_velocity_);
}

INSTANTIATE_TEST_CASE_P(VelocityPlanner, AccelerationSpecFixture,
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

    EXPECT_EQ(actual, ego_decelerated_velocity_);
}

INSTANTIATE_TEST_CASE_P(VelocityPlanner, DecelerationSpecFixture,
                        ::testing::Combine(::testing::Values(units::velocity::meters_per_second_t{0.0},
                                                             units::velocity::meters_per_second_t{15.0},
                                                             units::velocity::meters_per_second_t{17.0}),
                                           ::testing::Values(GlobalLaneId::kCenter)));

}  // namespace