///
/// @file
/// @brief Contains command line interface options definitions
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#ifndef PLANNING_COMMON_ARGUMENT_PARSER_CLI_OPTIONS_H_
#define PLANNING_COMMON_ARGUMENT_PARSER_CLI_OPTIONS_H_

#include <cstdint>
#include <string>

namespace planning
{
/// @brief Contains Command Line Interface (CLI) Options
struct CLIOptions
{
    /// @brief Path to map data
    std::string map_name = "data/highway_map.csv";

    /// @brief Enable/Disable Verbose Logging (Prints more information)
    bool verbose = false;
};

}  // namespace planning
#endif  /// PLANNING_COMMON_ARGUMENT_PARSER_CLI_OPTIONS_H_
