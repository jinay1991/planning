///
/// @file
///
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

#include <timer/chrono_timer.h>

namespace timer
{
namespace
{
class ChronoTimerSpecFixture : public ::testing::TestWithParam<std::chrono::system_clock::duration>
{
  protected:
    ChronoTimer timer_{};
};
TEST_P(ChronoTimerSpecFixture, GivenTypicalDuration_WhenStartTimer_ThenTimeoutOnDuration)
{
    const auto duration = GetParam();
    timer_.SetTimer(duration);
    timer_.Start();
    ASSERT_TRUE(timer_.IsRunning());
    ASSERT_FALSE(timer_.IsTimeout());

    std::this_thread::sleep_for(duration);

    EXPECT_FALSE(timer_.IsRunning());
    EXPECT_TRUE(timer_.IsTimeout());
}
INSTANTIATE_TEST_CASE_P(BoundaryValueCheck, ChronoTimerSpecFixture,
                        ::testing::Values(std::chrono::seconds{1}, std::chrono::milliseconds{500}));
}  // namespace
}  // namespace timer