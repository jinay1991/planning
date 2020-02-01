///
/// @file chrono_timer.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef TIMER_TIMER_H_
#define TIMER_TIMER_H_

#include "timer/i_timer.h"

namespace timer
{
class ChronoTimer : public ITimer
{
  public:
    /// @brief Constructor.
    ChronoTimer();

    /// @brief Start of the Timer
    virtual void Start() override;

    /// @brief Interrupt Timer
    virtual void Stop() override;

    /// @brief Check whether Timer has timeout (based on timeout value set)
    virtual bool IsTimeout() const override;

    /// @brief Check whether Time is running and is not timeout.
    virtual bool IsRunning() const override;

    /// @brief Set Timer Duration.
    virtual void SetTimer(const std::chrono::system_clock::duration& duration) override;

  private:
    /// @brief Timer Duration
    std::chrono::system_clock::duration duration_;

    /// @brief Start timepoint
    std::chrono::system_clock::time_point start_;

    /// @brief Status on timer started
    bool is_started_;
};
}  // namespace timer

#endif  // TIMER_TIMER_H_