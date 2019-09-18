///
/// @file
///

#include <motion_planning/motion_planning_fixture.h>

namespace
{
TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenGenerateTrajectories_ThenReturnSelectedTrajectory)
{
    // motion_planning_.GenerateTrajectories();
}

TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenGenerateManeuvers_ThenReturnThreeManeuvers) {}

TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenPlanTrajectories_ThenReturnThreePlannedTrajectories) {}

TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenEvaluatedTrajectories_ThenReturnEvalutatedTrajectories) {}

TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenPrioritizedTrajectories_ThenReturnPrioritizedTrajectories) {}

TEST_F(MotionPlanningFixture, GivenTypicalInputs_WhenSelectTrajectory_ThenReturnOnlyOneSelectedTrajectory) {}

}  // namespace
