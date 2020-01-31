///
/// @file
///
#ifndef TIMER_TIMER_H_
#define TIMER_TIMER_H_

#include "timer/i_timer.h"

namespace timer
{
class ChronoTimer : public ITimer
{
  public:
    virtual void Start() override;
    virtual void Stop() override;

    virtual bool IsTimeout() const override;
    virtual bool IsRunning() const override;

    virtual void SetTimer(const std::chrono::system_clock::duration& duration) override;

  private:
    std::chrono::system_clock::duration duration_{std::chrono::seconds{1}};
    std::chrono::system_clock::time_point start_;
    bool is_started_{false};
};
}  // namespace timer

#endif  // TIMER_TIMER_H_