///
/// @file
///

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <motion_planning/domain_model/lane.h>
#include <motion_planning/velocity_planner/velocity_planner.h>

#include "data_source_builder.h"

using namespace motion_planning;

namespace
{
using LaneId = LaneInformation::LaneId;
using GlobalLaneId = LaneInformation::GlobalLaneId;

class DecelerationSpecFixture : public ::testing::TestWithParam<units::velocity::meters_per_second_t>
{
  protected:
    virtual void SetUp() override
    {
        obj_velocity_ = GetParam();

        unit_ = std::make_unique<VelocityPlanner>(DataSourceBuilder()
                                                      .WithVelocity(ego_velocity_)
                                                      .WithGlobalLaneId(ego_lane_id_)
                                                      .WithObjectInLane(obj_lane_id_, obj_velocity_)
                                                      .Build());

        unit_->CalculateTargetVelocity();
    }

  public:
    const units::velocity::meters_per_second_t ego_velocity_{36.11};
    const GlobalLaneId ego_lane_id_{GlobalLaneId::kCenter};

    units::velocity::meters_per_second_t obj_velocity_{0.0};
    GlobalLaneId obj_lane_id_{GlobalLaneId::kCenter};

  protected:
    std::unique_ptr<VelocityPlanner> unit_;
};

TEST_P(DecelerationSpecFixture, GivenTypicalDataSource_WhenCalculateTargetVelocity_ThenUpdateTargetVelocity)
{
    const auto actual = unit_->GetTargetVelocity();

    EXPECT_LT(actual, ego_velocity_);
    EXPECT_EQ(actual, obj_velocity_);
}

INSTANTIATE_TEST_CASE_P(VelocityPlanner, DecelerationSpecFixture,
                        ::testing::Values(units::velocity::meters_per_second_t{0.0},
                                          units::velocity::meters_per_second_t{10.0},
                                          units::velocity::meters_per_second_t{20.0},
                                          units::velocity::meters_per_second_t{30.0},
                                          units::velocity::meters_per_second_t{36.11}));
}  // namespace