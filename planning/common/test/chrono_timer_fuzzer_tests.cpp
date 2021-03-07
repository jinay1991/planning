///
/// @file
/// @copyright Copyright (c) 2021. MIT License.
///
#include "planning/common/chrono_timer.h"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <sstream>

namespace timer
{
namespace
{
using namespace std::chrono_literals;

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* input, std::size_t size)
{
    std::istringstream s{std::string(reinterpret_cast<const char*>(input), size)};
    timer::ChronoTimer timer{};

    std::chrono::system_clock::duration duration{0ms};
    if (s.read(reinterpret_cast<char*>(&duration), sizeof(std::chrono::system_clock::duration)))
    {
        timer.SetTimer(duration);
    }

    return 0;
}
}  // namespace
}  // namespace timer
