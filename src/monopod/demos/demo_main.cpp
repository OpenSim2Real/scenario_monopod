#include <scenario/monopod/Joint.h>
#include <scenario/monopod/Model.h>
#include <scenario/monopod/World.h>
// #include <monopod_sdk/monopod.hpp>


#include <chrono>
#include <string>
#include <thread>
#include <iostream>

int main(int argc, char* argv[])
{
//     // Create the simulator
//     scenario::monopod::World world;
//     std::cout << world.name()
//               << std::endl;
//
//     auto modelNames = world.modelNames();
//     std::cout << modelNames[0]
//               << std::endl;
//
//     auto monopod = world.getModel(modelNames[0]);
//     std::cout << monopod->name()
//               << std::endl;
//
//     auto jointNames = monopod->jointNames(true);
//     for (std::string i: jointNames)
//         std::cout << i << ", ";
//     std::cout << std::endl << std::endl;
//
//     std::vector<double> forces = {1,2,3,4,5};
//     std::vector<std::string> jNames = {"upper_leg_joint", "lower_leg_joint",
//                                       "boom_pitch_joint", "boom_yaw_joint", "hip_joint"};
//
// // Check if setting force targets work before setting the control mode.
//
//     auto ok = monopod->setJointGeneralizedForceTargets(forces, jNames);
//     std::cout << "Setting force successful? "
//               << ok
//               << std::endl<< std::endl;
//
//     std::cout << "What are the set Joint Generalized force targets: ";
//     for (auto i: monopod->jointGeneralizedForceTargets())
//         std::cout << i << ", ";
//     std::cout << std::endl;
//
// // Set control mode and test again...
//
//     std::cout << "Setting all joints force controlMode successful? "
//               << monopod->setJointControlMode(scenario::core::JointControlMode::Force, jNames)
//               << std::endl<< std::endl;
//
//     ok = monopod->setJointGeneralizedForceTargets(forces, jNames);
//     std::cout << "Setting force successful? "
//               << ok
//               << std::endl << std::endl;
//
//     std::cout << "What are the set Joint Generalized force targets: ";
//     for (auto i: monopod->jointGeneralizedForceTargets(jNames))
//         std::cout << i << ", ";
//     std::cout << std::endl << std::endl;
//
// // Test the order of the force targets getter given specified joints
//
//     jNames = {"upper_leg_joint", "lower_leg_joint"};
//     std::cout << "For the joints: ";
//     for (auto i: jNames)
//         std::cout << i << ", ";
//     std::cout << std::endl << "What are the set Joint Generalized force targets? ";
//     for (auto i: monopod->jointGeneralizedForceTargets(jNames))
//         std::cout << i << ", ";
//     std::cout << std::endl << std::endl;
//
//   // Test vel, accel, pos getters
//
//   jNames = {"upper_leg_joint", "lower_leg_joint",
//                                     "boom_pitch_joint", "boom_yaw_joint", "hip_joint"};
//
//   auto pos = monopod->jointPositions(jNames);
//   std::cout << std::endl << "Joints position: ";
//   for (auto i: pos)
//       std::cout << i << ", ";
//   std::cout << std::endl << std::endl;
//
//   auto vel = monopod->jointVelocities(jNames);
//   std::cout << std::endl << "Joints Velocity ";
//   for (auto i: vel)
//       std::cout << i << ", ";
//   std::cout << std::endl << std::endl;
//
//   auto acc = monopod->jointAccelerations(jNames);
//   std::cout << std::endl << "Joints Acceleration ";
//   for (auto i: acc)
//       std::cout << i << ", ";
//   std::cout << std::endl << std::endl;

    return 0;
}
