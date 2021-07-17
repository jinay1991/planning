///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#include "middleware/communication/intra_process_pub_sub_factory.h"
#include "planning/motion_planning/node/motion_planning_node.h"
#include "planning/motion_planning/test/support/motion_planning_consumer_node.h"
#include "planning/motion_planning/test/support/motion_planning_simulator_node.h"

#include <benchmark/benchmark.h>

namespace planning
{

class MotionPlanningBenchmarkFixture : public ::benchmark::Fixture
{
  public:
    MotionPlanningBenchmarkFixture() : factory_{}, unit_{factory_}, consumer_{factory_}, simulator_{factory_} {}

  protected:
    void SetUp(const benchmark::State& /* state */) override
    {
        simulator_.Init();
        unit_.Init();
        consumer_.Init();
    }

    void TearDown(const benchmark::State& /* state */) override
    {
        simulator_.Shutdown();
        unit_.Shutdown();
        consumer_.Shutdown();
    }

    void RunBenchmarkTest(benchmark::State& state)
    {
        for (auto _ : state)
        {
            state.PauseTiming();
            simulator_.Step();
            state.ResumeTiming();

            unit_.Step();

            state.PauseTiming();
            consumer_.Step();
            state.ResumeTiming();
        }
    }

    void BlockEgoLane() { simulator_.BlockEgoLane(); }

  private:
    middleware::IntraProcessPubSubFactory factory_;
    MotionPlanningNode unit_;
    MotionPlanningConsumerNode consumer_;
    MotionPlanningSimulatorNode simulator_;
};

BENCHMARK_F(MotionPlanningBenchmarkFixture, DrivingInBlockedEgoLane)(benchmark::State& state)
{
    BlockEgoLane();
    RunBenchmarkTest(state);
}
}  // namespace planning