///
/// @file
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_COMMON_TIMER_I_TIMER_H_
#define PLANNING_COMMON_TIMER_I_TIMER_H_

#include <chrono>

namespace timer
{
/// @brief Interface for Timer
class ITimer
{
  public:
    /// @brief Destructor.
    virtual ~ITimer() = default;

    /// @brief Start of the Timer
    virtual void Start() = 0;

    /// @brief Interrupt Timer
    virtual void Stop() = 0;

    /// @brief Check whether Timer has timeout (based on timeout value set)
    virtual bool IsTimeout() const = 0;

    /// @brief Check whether Time is running and is not timeout.
    virtual bool IsRunning() const = 0;

    /// @brief Set Timer Duration.
    virtual void SetTimer(const std::chrono::system_clock::duration& duration) = 0;
};
}  // namespace timer

#endif  // PLANNING_COMMON_TIMER_I_TIMER_H_
