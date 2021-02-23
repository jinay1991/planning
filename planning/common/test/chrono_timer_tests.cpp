///
/// @file
/// @brief Contains unit tests for Chrono Timer.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/common/chrono_timer.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <thread>

namespace timer
{
namespace
{
class ChronoTimerFixture : public ::testing::TestWithParam<std::chrono::system_clock::duration>
{
  protected:
    ChronoTimer timer_{};
};

INSTANTIATE_TEST_SUITE_P(ChronoTimer,
                         ChronoTimerFixture,
                         ::testing::Values(std::chrono::seconds{1}, std::chrono::milliseconds{500}));

TEST_P(ChronoTimerFixture, Start_GivenTypicalDuration_ExpectTimeoutOnDuration)
{
    // Given
    const auto duration = GetParam();
    timer_.SetTimer(duration);

    // When
    timer_.Start();

    // Then
    ASSERT_TRUE(timer_.IsRunning());
    ASSERT_FALSE(timer_.IsTimeout());

    std::this_thread::sleep_for(duration);

    EXPECT_FALSE(timer_.IsRunning());
    EXPECT_TRUE(timer_.IsTimeout());
}

TEST_F(ChronoTimerFixture, Stop_GivenTypicalTimer_ExpectStoppedTimer)
{
    // Given
    timer_.SetTimer(std::chrono::seconds(10));
    timer_.Start();
    ASSERT_TRUE(timer_.IsRunning());
    ASSERT_FALSE(timer_.IsTimeout());

    // When
    timer_.Stop();

    // Then
    EXPECT_FALSE(timer_.IsRunning());
}
}  // namespace
}  // namespace timer
