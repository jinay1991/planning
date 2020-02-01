///
/// @file main.cpp
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include <iostream>
#include <memory>

#include "argument_parser/argument_parser.h"
#include "simulator/i_simulator.h"
#include "simulator/udacity_simulator.h"

int main(int argc, char* argv[])
{
    try
    {
        std::unique_ptr<planning::IArgumentParser> argument_parser =
            std::make_unique<planning::ArgumentParser>(argc, argv);
        auto cli_options = argument_parser->GetParsedArgs();
        std::unique_ptr<sim::ISimulator> sim = std::make_unique<sim::UdacitySimulator>(cli_options.map_name);
        sim->Init();

        sim->Listen();
    }
    catch (std::exception& e)
    {
        std::cout << "Failed to run simulator_client!! " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
