///
/// @file
///
#include "timer/chrono_timer.h"

namespace timer
{
void ChronoTimer::Start()
{
    start_ = std::chrono::system_clock::now();
    is_started_ = true;
}

void ChronoTimer::Stop()
{
    if (IsRunning())
    {
        is_started_ = false;
    }
}

bool ChronoTimer::IsRunning() const { return is_started_ && !IsTimeout(); }

bool ChronoTimer::IsTimeout() const { return is_started_ && ((std::chrono::system_clock::now() - start_) > duration_); }

void ChronoTimer::SetTimer(const std::chrono::system_clock::duration& duration) { duration_ = duration; }

}  // namespace timer