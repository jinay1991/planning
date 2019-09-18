///
/// @file
///

#ifndef MOTION_PLANNING_FIXTURE_H_
#define MOTION_PLANNING_FIXTURE_H_

#include <gtest/gtest.h>
#include <motion_planning/domain_model/trajectory.h>
#include <motion_planning/motion_planning.h>

namespace
{
class MotionPlanningFixture : public ::testing::Test
{
  public:
  protected:
    virtual void SetUp() override {}
    virtual void TearDown() override {}

    std::unique_ptr<motion_planning::MotionPlanning> motion_planning_;
};

}  // namespace
#endif  /// MOTION_PLANNING_FIXTURE_H
