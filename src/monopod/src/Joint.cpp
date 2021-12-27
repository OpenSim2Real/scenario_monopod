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

    switch (mode) {
        case core::JointControlMode::Force:
        {
            // Check if this joint can be controlled
            if (pImpl->monopod_sdk->is_joint_controllable(pImpl->monopodSdkIndex))
            {
              pImpl->jointControlMode = mode;
              // Set the force targets to 0 for all DOF...
              std::vector<double> forcetarget(this->dofs(), 0);
              this->setJointGeneralizedForceTarget(forcetarget);
              return true;
            }
            throw std::invalid_argument("Joint '" + this->name() + "' is not controllable. Does not support Force control mode.");

        }
        case core::JointControlMode::Invalid:
        case core::JointControlMode::Idle:
        {
            pImpl->jointControlMode = mode;
            return true;
        }
        default:
            // Real robot only has torque control or invalid (no control)
            throw std::invalid_argument("Only support force control mode.");
    }

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
        return jointPosition;
    }

    throw std::invalid_argument( "No joint position found for joint '" + this->name() + "'.");
    // return jointPosition;

}

std::vector<double> Joint::jointVelocity() const
{
    std::vector<double> jointVelocity;
    if(auto data = pImpl->monopod_sdk->get_velocity(pImpl->monopodSdkIndex))
    {
        jointVelocity.push_back(data.value());
        LOG(INFO) << "Getting velocity for joint, " + this->name() + " = " + std::to_string(data.value());
        return jointVelocity;
    }

    throw std::invalid_argument( "No joint velocity found for joint '" + this->name() + "'.");
    // return jointVelocity;

}

std::vector<double> Joint::jointAcceleration() const
{
    std::vector<double> jointAcceleration;
    if(auto data = pImpl->monopod_sdk->get_acceleration(pImpl->monopodSdkIndex))
    {
        jointAcceleration.push_back(data.value());
        LOG(INFO) << "Getting acceleration for joint, " + this->name() + " = " + std::to_string(data.value());
        return jointAcceleration;
    }

    throw std::invalid_argument( "No joint acceleration found for joint '" + this->name() + "'.");
    // return jointAcceleration;

}

std::vector<double> Joint::jointGeneralizedForceTarget() const
{

    std::vector<double> torqueTarget;

    if(this->controlMode() != core::JointControlMode::Force) {
        LOG(ERROR) << "Joint, '" + this->name() + "' is not in force control mode.";
        throw std::invalid_argument( "Joint, '" + this->name() + "' is not in force control mode.");
        // return torqueTarget;
    }

    if(auto data_op = pImpl->monopod_sdk->get_torque_target(pImpl->monopodSdkIndex))
    {
        torqueTarget.push_back(data_op.value());
        return torqueTarget;
    }

    throw std::invalid_argument( "No generalized force target found for joint '" + this->name() + "'.");
    // return torqueTarget;
}

bool Joint::setJointGeneralizedForceTarget(const std::vector<double>& force)
{

    if(this->controlMode() != core::JointControlMode::Force) {
        LOG(ERROR) << "Joint, '" + this->name() + "' is not in force control mode.";
        throw std::invalid_argument( "Joint, '" + this->name() + "' is not in force control mode.");
        // return false;
    }

    // Set the component data
    if (force.size() != this->dofs()) {
        LOG(ERROR) << "Wrong number of elements (joint_dofs=" + std::to_string(this->dofs()) + ")";
        throw std::invalid_argument( "Wrong number of elements (joint_dofs=" + std::to_string(this->dofs()) + ")" );
        // return false;
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
        return maxGeneralizedForce;
    }

    throw std::invalid_argument( "No max generalized force found for joint '" + this->name() + "'.");
    // return maxGeneralizedForce;
}

bool Joint::setJointMaxGeneralizedForce(const std::vector<double>& maxForce)
{
    if (maxForce.size() != this->dofs()) {
        LOG(ERROR) << "Wrong number of elements (joint_dofs=" + std::to_string(this->dofs()) + ")";
        throw std::invalid_argument( "Wrong number of elements (joint_dofs=" + std::to_string(this->dofs()) + ")" );
        // return false;
    }

    return pImpl->monopod_sdk->set_max_torque_target(maxForce[0], pImpl->monopodSdkIndex);

}

scenario::core::PID Joint::pid() const
{
    if(this->controlMode() != core::JointControlMode::Force) {
        LOG(ERROR) << "Joint, '" + this->name() + "' is not in force control mode. Does not support PID.";
        throw std::invalid_argument( "Joint, '" + this->name() + "' is not in force control mode. Does not support PID.");
    }

    if(auto data_op = pImpl->monopod_sdk->get_pid(pImpl->monopodSdkIndex)){
        auto data = data_op.value();
        return scenario::core::PID(data.p, data.i, data.d);
          // Set to default value here.
    }

    throw std::invalid_argument( "No PID value found for joint '" + this->name() + "'.");
    // return scenario::core::PID();

}

bool Joint::setPID(const scenario::core::PID& pid)
{
    if(this->controlMode() != core::JointControlMode::Force) {
        LOG(ERROR) << "Joint, '" + this->name() + "' is not in force control mode. Does not support PID.";
        throw std::invalid_argument( "Joint, '" + this->name() + "' is not in force control mode. Does not support PID.");
    }

    return pImpl->monopod_sdk->set_pid(pid.p, pid.i, pid.d, pImpl->monopodSdkIndex);
}

scenario::core::JointLimit Joint::jointPositionLimit() const
{
    if(auto data_op = pImpl->monopod_sdk->get_joint_position_limit(pImpl->monopodSdkIndex)){
        auto data = data_op.value();
        return scenario::core::JointLimit({data.min}, {data.max});
    }

    throw std::invalid_argument( "No position limit found for joint '" + this->name() + "'.");
    // return scenario::core::JointLimit(this->dofs());
}

scenario::core::JointLimit Joint::jointVelocityLimit() const
{
    if(auto data_op = pImpl->monopod_sdk->get_joint_velocity_limit(pImpl->monopodSdkIndex)){
        auto data = data_op.value();
        return scenario::core::JointLimit({data.min}, {data.max});
    }

    throw std::invalid_argument( "No velocity limit found for joint '" + this->name() + "'.");
    // return scenario::core::JointLimit(this->dofs());
}

scenario::core::JointLimit Joint::jointAccelerationLimit() const
{
    if(auto data_op = pImpl->monopod_sdk->get_joint_acceleration_limit(pImpl->monopodSdkIndex)){
        auto data = data_op.value();
        return scenario::core::JointLimit({data.min}, {data.max});
    }

    throw std::invalid_argument( "No acceleration limit found for joint '" + this->name() + "'.");
    // return scenario::core::JointLimit(this->dofs());

}


bool Joint::setJointPositionLimit(const double& maxPosition, const double& minPosition)
{
  return pImpl->monopod_sdk->set_joint_position_limit(maxPosition, minPosition, pImpl->monopodSdkIndex);
}

bool Joint::setJointVelocityLimit(const double& maxVelocity, const double& minVelocity)
{
  return pImpl->monopod_sdk->set_joint_velocity_limit(maxVelocity, minVelocity, pImpl->monopodSdkIndex);
}

bool Joint::setJointAccelerationLimit(const double& maxAcceleration, const double& minAcceleration)
{
  return pImpl->monopod_sdk->set_joint_acceleration_limit(maxAcceleration, minAcceleration, pImpl->monopodSdkIndex);
}
