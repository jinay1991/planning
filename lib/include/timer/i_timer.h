///
/// @file i_timer.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef TIMER_I_TIMER_H_
#define TIMER_I_TIMER_H_

#include <chrono>

namespace timer
{
class ITimer
{
  public:
    virtual void Start() = 0;
    virtual void Stop() = 0;

    virtual bool IsTimeout() const = 0;
    virtual bool IsRunning() const = 0;

    virtual void SetTimer(const std::chrono::system_clock::duration& duration) = 0;
};
}  // namespace timer

#endif  // TIMER_I_TIMER_H_