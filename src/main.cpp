///
/// @file main.cpp
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include <iostream>
#include <memory>

#include "argument_parser/argument_parser.h"
#include "simulation/simulation.h"

int main(int argc, char* argv[])
{
    try
    {
        std::unique_ptr<planning::IArgumentParser> argument_parser =
            std::make_unique<planning::ArgumentParser>(argc, argv);
        auto cli_options = argument_parser->GetParsedArgs();
        sim::Simulation sim{cli_options.map_name};
        sim.Run();
    }
    catch (std::exception& e)
    {
        std::cout << "Failed to run client-app!! " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
