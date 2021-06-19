///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#include "middleware/communication/intra_process_pub_sub_factory.h"
#include "planning/motion_planning/node/motion_planning_node.h"

#include <benchmark/benchmark.h>

namespace planning
{

class MotionPlanningBenchmarkFixture : public ::benchmark::Fixture
{
  public:
    MotionPlanningBenchmarkFixture() : factor_{}, unit_{factor_} {}

  protected:
    void SetUp(const benchmark::State& /* state */) override { unit_.Init(); }
    void TearDown(const benchmark::State& /* state */) override { unit_.Shutdown(); }

    void RunBenchmarkTest(benchmark::State& state)
    {
        for (auto _ : state)
        {
            state.PauseTiming();

            state.ResumeTiming();

            unit_.Step();
        }
    }

  private:
    middleware::IntraProcessPubSubFactory factor_;
    MotionPlanningNode unit_;
};

}  // namespace planning