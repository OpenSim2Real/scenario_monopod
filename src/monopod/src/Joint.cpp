#include "scenario/monopod/Joint.h"
#include "scenario/monopod/Model.h"
#include "scenario/monopod/World.h"

#include <limits>
#include <cassert>

using namespace scenario::monopod;
const scenario::core::PID DefaultPID;

class Joint::Impl
{
public:
    // We only have Revolute joints
    core::JointType jointType = core::JointType::Revolute;
    std::vector<double> forceTarget;
    std::vector<double> jointMaxGeneralizedForce;
    std::vector<double> jointControlMode = core::JointControlMode::Idle;
    string name;
};

Joint::Joint()
    : pImpl{std::make_unique<Impl>()}
{}

Joint::~Joint() = default;JointPtr

uint64_t Joint::id() const
{
    // Build a unique string identifier of this joint
    const std::string scopedJointName =
        "real_world::" + this->name(/*scoped=*/true);

    // Return the hashed string
    return std::hash<std::string>{}(scopedJointName);
}

bool Joint::initialize(const string _name)
{
    auto pImpl->name = _name;
    std::vector<double> pImpl->forceTarget(this->dofs(), 0)

    if (this->dofs() > 1) {
        sError << "Joints with DoFs > 1 are not currently supported"
               << std::endl;
        return false;
    }
    // Todo::Set default PID here.
    return true;
}

bool Model::valid() const
{
    // Hardware check here...
    bool ok = true;
    return ok;
}

scenario::core::JointType Joint::type() const
{
  const core::JointType type = pImpl->jointType;
  return type;
}

size_t Joint::dofs() const
{
    switch (this->type()) {
        case core::JointType::Invalid:
            return 0;
        case core::JointType::Fixed:
        case core::JointType::Revolute:
        case core::JointType::Prismatic:
            return 1;
    }
    assert(false);
    return 0;
}

std::string Joint::name(const bool scoped) const
{
    if (scoped) {
        jointName = "monopod::" + this->name;
        // jointName = utils::getParentModel(*this)->name() + "::" + jointName;
    }
    return jointName;
}

bool Joint::setControlMode(const scenario::core::JointControlMode mode)
{
    // Real robot only has torque control or invalid (no control)
    switch (mode) {
        case core::JointControlMode::Force:
            pImpl->jointControlMode = mode;
            return true;
        case core::JointControlMode::Position:
        case core::JointControlMode::PositionInterpolated:
        case core::JointControlMode::Velocity:
        case core::JointControlMode::VelocityFollowerDart:
        case core::JointControlMode::Idle:
        case core::JointControlMode::Invalid:
            sError << "Only support force control mode."
                   << std::endl;
            return false;
    }
}

scenario::core::JointControlMode Joint::controlMode() const
{
    const core::JointControlMode mode = pImpl->jointControlMode;
    return mode;
}

scenario::core::PID Joint::pid() const
{
    // get PID of low level controller here
    return NULL;
}

bool Joint::setPID(const scenario::core::PID& pid)
{
    // Set PID of lowlevel controller here
    return true;
}

std::vector<double> Joint::jointPosition() const
{
    const std::vector<double> jointPosition(this->dofs(), 0);
    // get joint pos from low level control

    if (jointPosition.size() != this->dofs()) {
        sError << "The size of velocity being set for the joint '" +
               << this->name() + "' does not match the joint's DOFs."
               << std::endl;
    }

    return jointPosition;
}

std::vector<double> Joint::jointVelocity() const
{
    const std::vector<double> jointVelocity(this->dofs(), 0);
    // Get joint Velocity from low level control

    if (jointVelocity.size() != this->dofs()) {
        sError << "The size of velocity being set for the joint '" +
               << this->name() + "' does not match the joint's DOFs."
               << std::endl;
    }

    return jointVelocity;
}

std::vector<double> Joint::jointAcceleration() const
{
    const std::vector<double> jointAcceleration(this->dofs(), 0);
    // Get joint acceleration from low level control

    if (jointAcceleration.size() != this->dofs()) {
        sError << "The size of acceleration being set for the joint '" +
               << this->name() + "' does not match the joint's DOFs."
               << std::endl;
    }

    return jointAcceleration;
}

std::vector<double> Joint::jointGeneralizedForceTarget() const
{
    return pImpl->forceTarget;
}

bool Joint::setJointGeneralizedForceTarget(const std::vector<double>& force)
{
    if (force.size() != this->dofs()) {
        sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
               << std::endl;
        return false;

    }

    const std::vector<double>& maxForce = this->jointMaxGeneralizedForce();
    for (size_t dof = 0; dof < this->dofs(); ++dof) {
        if (std::abs(force[dof]) > maxForce[dof]) {
            sWarning << "The force target is higher than the limit. "
                     << "The physics engine might clip it." << std::endl;
        }
    }

    // Set the component data
    pImpl->forceTarget = force;
    return true;
}

std::vector<double> Joint::jointMaxGeneralizedForce() const
{
    std::vector<double> maxGeneralizedForce;
    switch (this->type()) {
        case core::JointType::Revolute: {
            maxGeneralizedForce = pImpl->jointMaxGeneralizedForce
            break;
        }
        case core::JointType::Fixed:
        case core::JointType::Prismatic:
        case core::JointType::Invalid:
        case core::JointType::Ball:{
            sWarning << "Type of Joint with name '" << this->name()
                     << "' has no max effort defined" << std::endl;
            break;
        }
    }
    return maxGeneralizedForce;
}

bool Joint::setJointMaxGeneralizedForce(const std::vector<double>& maxForce)
{

    if (maxForce.size() != this->dofs()) {
        sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
               << std::endl;
        return false;
    }

    assert(maxForce.size() == 1);
    pImpl->jointMaxGeneralizedForce = maxForce
    return true;
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
