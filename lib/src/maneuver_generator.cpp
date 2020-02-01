///
/// @file maneuver_generator.cpp
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include <algorithm>
#include <sstream>

#include "logging/logging.h"
#include "motion_planning/maneuver_generator.h"

namespace motion_planning
{
std::vector<Maneuver> ManeuverGenerator::Generate(const units::velocity::meters_per_second_t& target_velocity) const
{
    auto maneuvers =
        std::vector<Maneuver>{Maneuver{LaneId::kLeft, target_velocity}, Maneuver{LaneId::kEgo, target_velocity},
                              Maneuver{LaneId::kRight, target_velocity}};

    std::stringstream log_stream;
    log_stream << "Generated Maneuvers:" << std::endl;
    std::for_each(maneuvers.begin(), maneuvers.end(),
                  [&](const auto& maneuver) { log_stream << " (+) " << maneuver << std::endl; });
    LOG(DEBUG) << log_stream.str();
    return maneuvers;
}
}  // namespace motion_planning
