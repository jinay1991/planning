///
/// @file
///
#include <simulation/simulation.h>

int main(int argc, char* argv[])
{
    sim::Simulation sim{argv[1]};
    sim.Run();
    return 0;
}
