///
/// @file
/// @brief Contains Argument Parser definitions
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#ifndef PLANNING_COMMON_ARGUMENT_PARSER_ARGUMENT_PARSER_H
#define PLANNING_COMMON_ARGUMENT_PARSER_ARGUMENT_PARSER_H

#include "planning/common/cli_options.h"
#include "planning/common/i_argument_parser.h"

#include <getopt.h>
#include <unistd.h>

#include <string>
#include <vector>

namespace planning
{
/// @brief Argument Parser class
class ArgumentParser : public IArgumentParser
{
  public:
    /// @brief Default Constructor
    ArgumentParser();

    /// @brief Constructor
    /// @param [in] argc - number of arguments
    /// @param [in] argv - list of arguments
    explicit ArgumentParser(int argc, char* argv[]);

    /// @brief Destructor
    ~ArgumentParser();

    /// @brief Provides Parsed Arguments
    CLIOptions GetParsedArgs() const override;

  protected:
    /// @brief Parse Arguments from argc, argv
    CLIOptions ParseArgs(int argc, char* argv[]) override;

  private:
    /// @brief parsed arguments
    CLIOptions cli_options_;

    /// @brief long option list
    std::vector<struct option> long_options_;

    /// @brief short option list
    std::string optstring_;
};

}  // namespace planning

#endif  /// PLANNING_COMMON_ARGUMENT_PARSER_ARGUMENT_PARSER_H
