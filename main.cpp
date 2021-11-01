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
    std::cout << modelNames[0]
              << std::endl;

    auto monopod = world.getModel(modelNames[0]);
    std::cout << monopod->name()
              << std::endl;

    auto jointNames = monopod->jointNames(true);
    for (std::string i: jointNames)
        std::cout << i << ", ";
    std::cout << std::endl;

    std::vector<double> forces = {1,2,3,4,5};
    std::vector<std::string> jNames = {"upper_leg_joint", "lower_leg_joint",
                                      "boom_pitch_joint", "boom_yaw_joint", "hip_joint"};
    auto ok = monopod->setJointGeneralizedForceTargets(forces);
    std::cout << "Setting force successful? "
              << ok
              << std::endl;

    ok = monopod->setJointGeneralizedForceTargets(forces, jNames);
    std::cout << "Setting force successful? "
              << ok
              << std::endl;

    std::cout << "What are the set Joint Generalized force targets: ";
    for (auto i: monopod->jointGeneralizedForceTargets())
        std::cout << i << ", ";
    std::cout << std::endl;
    return 0;
}
