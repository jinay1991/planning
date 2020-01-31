///
/// @file
///
#include <iostream>

#include "simulation/simulation.h"

int main(int argc, char* argv[])
{
    try
    {
        sim::Simulation sim{argv[1]};
        sim.Run();
    }
    catch (std::exception& e)
    {
        std::cout << "Failed to run client-app!! " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
