///
/// @file
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/maneuver_generator.h"

#include "planning/common/logging.h"

#include <algorithm>
#include <sstream>

namespace planning
{
std::vector<Maneuver> ManeuverGenerator::Generate(const units::velocity::meters_per_second_t& target_velocity) const
{
    auto maneuvers = std::vector<Maneuver>{Maneuver{LaneId::kLeft, target_velocity},
                                           Maneuver{LaneId::kEgo, target_velocity},
                                           Maneuver{LaneId::kRight, target_velocity}};

    std::stringstream log_stream;
    log_stream << "Generated Maneuvers:" << std::endl;
    std::for_each(maneuvers.begin(), maneuvers.end(), [&](const auto& maneuver) {
        log_stream << " (+) " << maneuver << std::endl;
    });
    LOG(INFO) << log_stream.str();
    return maneuvers;
}
}  // namespace planning
