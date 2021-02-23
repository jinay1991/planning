///
/// @file
/// @copyright Copyright (c) 2020-2021. All Rights Reserved.
///
#include "planning/common/argument_parser/argument_parser.h"
#include "planning/common/logging/logging.h"

namespace planning
{
namespace
{
void PrintUsage()
{
    LOG(INFO) << "Command Line Options: \n"
              << "--map_data, -m: path to map data\n"
              << "--verbose, -v: [0|1] print more information\n"
              << "--help, -h: print help\n";
}
}  // namespace
ArgumentParser::ArgumentParser() : cli_options_{} {}

ArgumentParser::ArgumentParser(int argc, char* argv[])
    : long_options_{{"map_data", required_argument, nullptr, 'm'},
                    {"verbose", required_argument, nullptr, 'v'},
                    {"help", 0, nullptr, 'h'},
                    {nullptr, 0, nullptr, 0}},
      optstring_{"h:m:v:"}
{
    cli_options_ = ParseArgs(argc, argv);
}

ArgumentParser::~ArgumentParser() {}

CLIOptions ArgumentParser::GetParsedArgs() const { return cli_options_; }

CLIOptions ArgumentParser::ParseArgs(int argc, char* argv[])
{
    while (true)
    {
        std::int32_t c = 0;
        std::int32_t optindex = 0;

        c = getopt_long(argc, argv, optstring_.c_str(), long_options_.data(), &optindex);
        // Detect the end of the options.
        if (c == -1)
        {
            break;
        }

        switch (c)
        {
            case 'm':
                cli_options_.map_name = optarg;
                LOG(INFO) << "map_name: " << cli_options_.map_name;
                break;
            case 'v':
                cli_options_.verbose = strtol(optarg, nullptr, 10);
                LOG(INFO) << "verbose: " << cli_options_.verbose;
                break;
            case 'h':
            case '?':
                // getopt_long already printed an error message.
                PrintUsage();
            default:
                exit(1);
        }
    }
    return cli_options_;
}

}  // namespace planning
