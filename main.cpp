#include <scenario/monopod/Joint.h>
#include <scenario/monopod/Model.h>
#include <scenario/monopod/World.h>

#include <chrono>
#include <string>
#include <thread>
#include <iostream>

int main(int argc, char* argv[])
{
    // Create the simulator
    scenario::monopod::World world;

    std::cout << world.name();
    return 0;
}
