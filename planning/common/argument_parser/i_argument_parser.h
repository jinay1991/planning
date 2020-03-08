///
/// @file i_argument_parser.h
/// @brief Contains Argument Parser Interface class
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef PLANNING_COMMON_ARGUMENT_PARSER_I_ARGUMENT_PARSER_H_
#define PLANNING_COMMON_ARGUMENT_PARSER_I_ARGUMENT_PARSER_H_

#include "planning/common/argument_parser/cli_options.h"

namespace planning
{
/// @brief Argument Parser Interface class
class IArgumentParser
{
  public:
    /// @brief Destructor
    virtual ~IArgumentParser() = default;

    /// @brief Provides Parsed Arguments
    virtual CLIOptions GetParsedArgs() const = 0;

  protected:
    /// @brief Parse Arguments from argc, argv
    virtual CLIOptions ParseArgs(int argc, char** argv) = 0;
};
}  // namespace planning

#endif  /// PLANNING_COMMON_ARGUMENT_PARSER_I_ARGUMENT_PARSER_H_