#include "scenario/monopod/Joint.h"
#include "scenario/monopod/Model.h"
#include "scenario/monopod/World.h"

#include <cassert>
#include <stdexcept>
#include <limits>

#include "scenario/monopod/easylogging++.h"



using namespace scenario::monopod;
const scenario::core::PID DefaultPID;

class Joint::Impl
{
public:
    // We only have Revolute joints
    const core::JointType jointType = core::JointType::Revolute;
    core::JointControlMode jointControlMode = core::JointControlMode::Idle;
    std::optional<std::vector<double>> jointMaxGeneralizedForce;
    std::string parentModelName;
    std::string name;
    int monopodSdkIndex;
    std::shared_ptr<monopod_drivers::Monopod> monopod_sdk;

};

Joint::Joint()
    : pImpl{std::make_unique<Impl>()}
{}

Joint::~Joint() = default;

uint64_t Joint::id() const
{
    // Build a unique string identifier of this joint
    const std::string scopedJointName =
        pImpl->parentModelName + "::" + this->name(/*scoped=*/true);

    // Return the hashed string
    return std::hash<std::string>{}(scopedJointName);
}

bool Joint::initialize(const std::pair<std::string, int> nameIndexPair,
                       const std::shared_ptr<monopod_drivers::Monopod> &monopod_sdk)
{
    // Set the names...
    pImpl->name = nameIndexPair.first;
    pImpl->monopodSdkIndex = nameIndexPair.second;
    pImpl->monopod_sdk = monopod_sdk;
    pImpl->parentModelName = pImpl->monopod_sdk->get_model_name();

    return true;
}

bool Joint::valid() const
{
    // TODO: Hardware check here...
    return true;
}

scenario::core::JointType Joint::type() const
{
    return pImpl->jointType;
}

size_t Joint::dofs() const
{
    // switch (this->type()) {
    //     case core::JointType::Fixed:
    //     case core::JointType::Revolute:
    //     case core::JointType::Prismatic:
    //         return 1;
    //     case core::JointType::Invalid:
    //         return 0;
    //     default:
    //         assert(false);
    //         return 0;
    // }
    return 1;
}

std::string Joint::name(const bool scoped) const
{
    std::string jointName;
    if (scoped) {
        jointName = pImpl->parentModelName + "::" + pImpl->name;
    }else{
        jointName = pImpl->name;
    }
    return jointName;
}

bool Joint::setControlMode(const scenario::core::JointControlMode mode)
{
    // Real robot only has torque control or invalid (no control)
    switch (mode) {
        case core::JointControlMode::Force:
        {
            pImpl->jointControlMode = mode;
            // Set the force targets to 0 for all DOF...
            std::vector<double> forcetarget(this->dofs(), 0);
            this->setJointGeneralizedForceTarget(forcetarget);
            return true;
        }
        default:
            LOG(ERROR) << "Only support force control mode.";
            return false;
    }
    return false;
}

scenario::core::JointControlMode Joint::controlMode() const
{
    return pImpl->jointControlMode;
}

std::vector<double> Joint::jointPosition() const
{
    std::vector<double> jointPosition;
    if(auto data = pImpl->monopod_sdk->get_position(pImpl->monopodSdkIndex))
    {
        jointPosition.push_back(data.value());
        LOG(INFO) << "Getting position for joint, " + this->name() + " = " + std::to_string(data.value());
    }

    return jointPosition;
}

std::vector<double> Joint::jointVelocity() const
{
    std::vector<double> jointVelocity;
    if(auto data = pImpl->monopod_sdk->get_velocity(pImpl->monopodSdkIndex))
    {
        jointVelocity.push_back(data.value());
        LOG(INFO) << "Getting velocity for joint, " + this->name() + " = " + std::to_string(data.value());
    }

    return jointVelocity;
}

std::vector<double> Joint::jointAcceleration() const
{
    std::vector<double> jointAcceleration;
    if(auto data = pImpl->monopod_sdk->get_acceleration(pImpl->monopodSdkIndex))
    {
        jointAcceleration.push_back(data.value());
        LOG(INFO) << "Getting acceleration for joint, " + this->name() + " = " + std::to_string(data.value());
    }

    return jointAcceleration;
}

std::vector<double> Joint::jointGeneralizedForceTarget() const
{

    std::vector<double> torque_target;

    if(this->controlMode() != core::JointControlMode::Force) {
        LOG(ERROR) << "Joint, '" + this->name() + "' is not in force control mode.";
        return torque_target;
    }

    if(auto data_op = pImpl->monopod_sdk->get_torque_target(pImpl->monopodSdkIndex))
    {
        torque_target.push_back(data_op.value());
    }

    return torque_target;
}

bool Joint::setJointGeneralizedForceTarget(const std::vector<double>& force)
{

    if(this->controlMode() != core::JointControlMode::Force) {
        LOG(ERROR) << "Joint, '" + this->name() + "' is not in force control mode.";
        return false;
    }

    // Set the component data
    if (force.size() != this->dofs()) {
        LOG(ERROR) << "Wrong number of elements (joint_dofs=" + std::to_string(this->dofs()) + ")";
        return false;
    }

    // // NOTE: This gets handled in monopod_sdk. No need to waste
    // // time getting the max generalized force
    // const std::vector<double>& maxForce = this->jointMaxGeneralizedForce();
    // for (size_t dof = 0; dof < this->dofs(); ++dof) {
    //     if (std::abs(force[dof]) > maxForce[dof]) {
    //         LOG(WARNING) << "The force target is higher than the limit. It may be clipped.";
    //     }
    // }

    return pImpl->monopod_sdk->set_torque_target(force[0], pImpl->monopodSdkIndex);
}

std::vector<double> Joint::jointMaxGeneralizedForce() const
{

    std::vector<double> maxGeneralizedForce;

    if(auto data_op = pImpl->monopod_sdk->get_max_torque_target(pImpl->monopodSdkIndex)){
        maxGeneralizedForce.push_back(data_op.value());
        // Set to default value here.
    }
    return maxGeneralizedForce;
}

bool Joint::setJointMaxGeneralizedForce(const std::vector<double>& maxForce)
{
    if (maxForce.size() != this->dofs()) {
        LOG(ERROR) << "Wrong number of elements (joint_dofs=" + std::to_string(this->dofs()) + ")";
        return false;
    }
    return pImpl->monopod_sdk->set_max_torque_target(maxForce[0], pImpl->monopodSdkIndex);

}

scenario::core::PID Joint::pid() const
{
    // get PID of low level controller here

    if(auto data_op = pImpl->monopod_sdk->get_pid(pImpl->monopodSdkIndex)){
        auto data = data_op.value();
        return scenario::core::PID(data.p, data.i, data.d);
          // Set to default value here.
    }else{
        return scenario::core::PID();
    }
}

bool Joint::setPID(const scenario::core::PID& pid)
{
    // Set PID of lowlevel controller here
    return pImpl->monopod_sdk->set_pid(pid.p, pid.i, pid.d, pImpl->monopodSdkIndex);
}

// scenario::core::JointLimit Joint::jointPositionLimit() const
// {
//     core::JointLimit jointLimit(this->dofs());
//
//     // switch (this->type()) {
//     //     case core::JointType::Revolute:
//     //     case core::JointType::Prismatic: {
//     //         sdf::JointAxis& axis = utils::getExistingComponentData< //
//     //             ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
//     //         jointLimit.min[0] = axis.Lower();
//     //         jointLimit.max[0] = axis.Upper();
//     //         break;
//     //     }
//     //     case core::JointType::Fixed:
//     //         sWarning << "Fixed joints do not have DOFs, limits are not defined"
//     //                  << std::endl;
//     //         break;
//     //     case core::JointType::Invalid:
//     //     case core::JointType::Ball:
//     //         sWarning << "Type of Joint '" << this->name() << "' has no limits"
//     //                  << std::endl;
//     //         break;
//     // }
//
//     return jointLimit;
// }
//
// scenario::core::JointLimit Joint::jointVelocityLimit() const
// {
//     core::JointLimit jointLimit(this->dofs());
//
//     // switch (this->type()) {
//     //     case core::JointType::Revolute:
//     //     case core::JointType::Prismatic: {
//     //         sdf::JointAxis& axis = utils::getExistingComponentData< //
//     //             ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
//     //         jointLimit.min[0] = -axis.MaxVelocity();
//     //         jointLimit.max[0] = axis.MaxVelocity();
//     //         break;
//     //     }
//     //     case core::JointType::Fixed:
//     //         sWarning << "Fixed joints do not have DOFs, limits are not defined"
//     //                  << std::endl;
//     //         break;
//     //     case core::JointType::Invalid:
//     //     case core::JointType::Ball:
//     //         sWarning << "Type of Joint '" << this->name() << "' has no limits"
//     //                  << std::endl;
//     //         break;
//     // }
//
//     return jointLimit;
// }
//
// scenario::core::JointLimit Joint::jointAccelerationLimit() const
// {
//     core::JointLimit jointLimit(this->dofs());
//
//     // switch (this->type()) {
//     //     case core::JointType::Revolute:
//     //     case core::JointType::Prismatic: {
//     //         sdf::JointAxis& axis = utils::getExistingComponentData< //
//     //             ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
//     //         jointLimit.min[0] = -axis.MaxAcceleration();
//     //         jointLimit.max[0] = axis.MaxAcceleration();
//     //         break;
//     //     }
//     //     case core::JointType::Fixed:
//     //         sWarning << "Fixed joints do not have DOFs, limits are not defined"
//     //                  << std::endl;
//     //         break;
//     //     case core::JointType::Invalid:
//     //     case core::JointType::Ball:
//     //         sWarning << "Type of Joint '" << this->name() << "' has no limits"
//     //                  << std::endl;
//     //         break;
//     // }
//
//     return jointLimit;
// }
//
// bool Joint::setJointVelocityLimit(const std::vector<double>& maxVelocity)
// {
//     if (maxVelocity.size() != this->dofs()) {
//         sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
//                << std::endl;
//         return false;
//     }
//
//     // switch (this->type()) {
//     //     case core::JointType::Revolute: {
//     //         sdf::JointAxis& axis = utils::getExistingComponentData< //
//     //             ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
//     //         axis.SetMaxVelocity(maxVelocity[0]);
//     //         return true;
//     //     }
//     //     case core::JointType::Ball:
//     //     case core::JointType::Prismatic:
//     //     case core::JointType::Fixed:
//     //     case core::JointType::Invalid:
//     //         sWarning << "Fixed and Invalid joints have no friction defined."
//     //                  << std::endl;
//     //         return false;
//     // }
//
//     return false;
// }
