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
    std::cout << world.name()
              << std::endl;

    auto modelNames = world.modelNames();
    std::cout << modelNames
              << std::endl;

    auto monopod = world.getModel(modelNames[0]);
    std::cout << monopod.name()
              << std::endl;
    return 0;
}
