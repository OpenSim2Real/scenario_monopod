#include "scenario/monopod/Model.h"
#include "scenario/monopod/Joint.h"
#include "scenario/monopod/World.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <functional>
#include <tuple>
#include <unordered_map>

using namespace scenario::monopod;

class Model::Impl
{
public:

    using JointName = std::string;
    std::unordered_map<JointName, core::JointPtr> joints;

    struct
    {
        std::optional<std::vector<std::string>> jointNames;
        std::optional<std::vector<std::string>> scopedJointNames;
    } buffers;

    static std::vector<double> getJointDataSerialized(
        const Model* model,
        const std::vector<std::string>& jointNames,
        std::function<double(core::JointPtr, const size_t)> getJointData);

    static bool setJointDataSerialized(
        Model* model,
        const std::vector<double>& data,
        const std::vector<std::string>& jointNames,
        std::function<bool(core::JointPtr, const double, const size_t)>
            setDataToDOF);
};

Model::Model()
    : pImpl{std::make_unique<Impl>()}
{}

Model::~Model() = default;

uint64_t Model::id() const
{
    // Build a unique string identifier of this model
    const std::string scopedModelName =
        "real_world::" + this->name();
    // Return the hashed string
    return std::hash<std::string>{}(scopedModelName);
}

bool Model::initialize()
{

    return true;
}

std::string Model::name() const
{
    std::string modelName = "monopod"
    return modelName;
}

scenario::core::JointPtr Model::getJoint(const std::string& jointName) const
{
    if (pImpl->joints.find(jointName) != pImpl->joints.end()) {
        assert(pImpl->joints.at(jointName));
        return pImpl->joints.at(jointName);
    }

    // Create the joint
    auto joint = std::make_shared<scenario::gazebo::Joint>();

    if (!joint->initialize(jointEntity, m_ecm, m_eventManager)) {
        throw exceptions::JointError("Failed to initialize joint", jointName);
    }

    // Cache the joint instance
    pImpl->joints[jointName] = joint;

    return joint;
}

std::vector<std::string> Model::jointNames(const bool scoped) const
{
    if (!scoped && pImpl->buffers.jointNames.has_value()) {
        return pImpl->buffers.jointNames.value();
    }
    if (scoped && pImpl->buffers.scopedLinkNames.has_value()) {
        return pImpl->buffers.scopedLinkNames.value();
    }

    // Make scoped joint names from joints if it does not exist.
    std::vector<std::string> jointNames;
    for(auto& itr : pImpl->Joints){
        prefix = this->name() + "::" + itr.second->name(scoped);
        jointNames.push_back(prefix);
      }
    if (scoped) {
        pImpl->buffers.scopedJointNames = std::move(jointNames);
        return pImpl->buffers.scopedJointNames.value();
    }else{
        pImpl->buffers.jointNames = std::move(jointNames);
        return pImpl->buffers.jointNames.value();
    }
}

std::vector<double>
Model::jointPositions(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->position(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double>
Model::jointVelocities(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->velocity(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double>
Model::jointAccelerations(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->acceleration(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

bool Model::setJointControlMode(const scenario::core::JointControlMode mode,
                                const std::vector<std::string>& jointNames)
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    bool ok = true;

    for (auto& joint : this->joints(jointSerialization)) {
        ok = ok && joint->setControlMode(mode);
    }

    return ok;
}

std::vector<scenario::core::JointPtr>
Model::joints(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    std::vector<core::JointPtr> joints;

    for (const auto& jointName : jointSerialization) {
        joints.push_back(this->getJoint(jointName));
    }

    return joints;
}


bool Model::setJointGeneralizedForceTargets(
    const std::vector<double>& forces,
    const std::vector<std::string>& jointNames)
{
    auto lambda =
        [](core::JointPtr joint, const double force, const size_t dof) -> bool {
        return joint->setGeneralizedForceTarget(force, dof);
    };

    return Impl::setJointDataSerialized(this, forces, jointNames, lambda);
}


std::vector<double> Model::jointGeneralizedForceTargets(
    const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->generalizedForceTarget(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}


// ======================
// Implementation Methods
// ======================

std::vector<double> Model::Impl::getJointDataSerialized(
    const Model* model,
    const std::vector<std::string>& jointNames,
    std::function<double(core::JointPtr, const size_t)> getJointData)
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? model->jointNames() : jointNames;

    std::vector<double> data;
    data.reserve(model->dofs());

    for (auto& joint : model->joints(jointSerialization)) {
        for (size_t dof = 0; dof < joint->dofs(); ++dof) {
            data.push_back(getJointData(joint, dof));
        }
    }

    return data;
}

bool Model::Impl::setJointDataSerialized(
    Model* model,
    const std::vector<double>& data,
    const std::vector<std::string>& jointNames,
    std::function<bool(core::JointPtr, const double, const size_t)>
        setJointData)
{
    std::vector<std::string> jointSerialization;

    size_t expectedDOFs = 0;

    // Set the Total DOF for jointnames passed into func.
    // otherwise do all joints
    if (!jointNames.empty()) {
        jointSerialization = jointNames;

        for (auto& joint : model->joints(jointSerialization)) {
            expectedDOFs += joint->dofs();
        }
    }
    else {
        expectedDOFs = model->dofs();
        jointSerialization = model->jointNames();
    }

    if (data.size() != expectedDOFs) {
        sError << "The size of value being set for each joint "
                  "does not match the considered joint's DOFs."
               << std::endl;
        return false;
    }

    auto it = data.begin();

    for (auto& joint : model->joints(jointNames)) {
        for (size_t dof = 0; dof < joint->dofs(); ++dof) {
            if (!setJointData(joint, *it++, dof)) {
                sError << "Failed to set force of joint '" << joint->name()
                       << "'" << std::endl;
                return false;
            }
        }
    }
    assert(it == data.end());
    return true;
}

// bool Model::resetJointPositions(const std::vector<double>& positions,
//                                 const std::vector<std::string>& jointNames)
// {
//     auto lambda = [](core::JointPtr joint,
//                      const double position,
//                      const size_t dof) -> bool {
//         return std::static_pointer_cast<Joint>(joint)->resetPosition(position,
//                                                                      dof);
//     };
//
//     return Impl::setJointDataSerialized(this, positions, jointNames, lambda);
// }
//
// bool Model::resetJointVelocities(const std::vector<double>& velocities,
//                                  const std::vector<std::string>& jointNames)
// {
//     auto lambda = [](core::JointPtr joint,
//                      const double velocity,
//                      const size_t dof) -> bool {
//         return std::static_pointer_cast<Joint>(joint)->resetVelocity(velocity,
//                                                                      dof);
//     };
//
//     return Impl::setJointDataSerialized(this, velocities, jointNames, lambda);
// }

// scenario::core::JointLimit
// Model::jointLimits(const std::vector<std::string>& jointNames) const
// {
//     const std::vector<std::string>& jointSerialization =
//         jointNames.empty() ? this->jointNames() : jointNames;
//
//     std::vector<double> low;
//     std::vector<double> high;
//
//     low.reserve(jointSerialization.size());
//     high.reserve(jointSerialization.size());
//
//     for (const auto& joint : this->joints(jointSerialization)) {
//         auto limit = joint->jointPositionLimit();
//         std::move(limit.min.begin(), limit.min.end(), std::back_inserter(low));
//         std::move(limit.max.begin(), limit.max.end(), std::back_inserter(high));
//     }
//
//     return core::JointLimit(low, high);
// }
