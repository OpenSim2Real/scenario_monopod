#include <scenario/monopod/Joint.h>
#include <scenario/monopod/Model.h>
#include <scenario/monopod/World.h>

#include <chrono>
#include <string>
#include <thread>

int main(int argc, char* argv[])
{
    // Create the simulator
    auto world = scenario::gazebo::world();

    // Initialize the simulator
    world.initialize();

    cout << world.name();
    return 0;
}
