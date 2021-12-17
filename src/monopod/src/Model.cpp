#include "scenario/monopod/Model.h"
#include "scenario/monopod/Joint.h"
#include "scenario/monopod/World.h"
#include "scenario/monopod/easylogging++.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <functional>
#include <tuple>
#include <unordered_map>
#include <stdexcept>


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
        std::function<std::vector<double>(core::JointPtr)> getJointData);

    static bool setJointDataSerialized(
        Model* model,
        const std::vector<double>& data,
        const std::vector<std::string>& jointNames,
        std::function<bool(core::JointPtr, const std::vector<double>)> setJointData);

    std::shared_ptr<monopod_drivers::Monopod> monopod_sdk;
};

Model::Model()
    : pImpl{std::make_unique<Impl>()}
{
    pImpl->monopod_sdk = std::make_shared<monopod_drivers::Monopod>();
    pImpl->monopod_sdk->initialize();
    pImpl->monopod_sdk->start_loop();

    std::vector<std::string> jointNames = pImpl->monopod_sdk->get_jointNames();

    // Set up all the joints.
    for (auto& jointName : jointNames) {
        // Initialize The joint
        // Will need to include the Robot SDK to be passed into the initialize
        auto joint = std::make_shared<scenario::monopod::Joint>();
        joint->initialize(jointName, this->name(), pImpl->monopod_sdk);

        // Cache joint
        pImpl->joints[jointName] = joint;
    }
    // Do not need to store the joint name.
    // It will be Cached the first time it is called.
}

Model::~Model() = default;

uint64_t Model::id() const
{
    // Build a unique string identifier of this model
    const std::string scopedModelName =
        "real_world::" + this->name();
    // Return the hashed string
    return std::hash<std::string>{}(scopedModelName);
}

bool Model::valid() const
{
    bool ok = true;
    for (auto& joint : this->joints(this->jointNames())) {
        ok = ok && joint->valid();
    }
    return ok;
}

std::string Model::name() const
{
    std::string modelName = "monopod";
    return modelName;
}

size_t Model::dofs(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    size_t dofs = 0;

    for (const auto& jointName : jointSerialization) {
        dofs += this->getJoint(jointName)->dofs();
    }

    return dofs;
}

std::vector<std::string> Model::jointNames(const bool scoped) const
{
    if (!scoped && pImpl->buffers.jointNames.has_value()) {
        return pImpl->buffers.jointNames.value();
    }
    if (scoped && pImpl->buffers.scopedJointNames.has_value()) {
        return pImpl->buffers.scopedJointNames.value();
    }

    std::vector<std::string> jointNames;
    for(auto itr : pImpl->joints){
        std::string prefix = itr.second->name(scoped);
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

scenario::core::JointPtr Model::getJoint(const std::string& jointName) const
{
    if (pImpl->joints.find(jointName) != pImpl->joints.end()) {
        assert(pImpl->joints.at(jointName));
        return pImpl->joints.at(jointName);
    }
    std::string str = " ";
    for(auto& name: this->jointNames())
        str = str + " " + name;

    LOG(ERROR) << "Joint name does not exist in model. Available models are: " + str;
    throw std::invalid_argument( "Joint name does not exist in model. Available models are: " + str );
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

std::vector<double>
Model::jointPositions(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint) -> std::vector<double>  {
        return joint->jointPosition();
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double>
Model::jointVelocities(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint) -> std::vector<double>  {
        return joint->jointVelocity();
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double>
Model::jointAccelerations(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint) -> std::vector<double>  {
        return joint->jointAcceleration();
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

bool Model::setJointGeneralizedForceTargets(
    const std::vector<double>& forces,
    const std::vector<std::string>& jointNames)
{
    auto lambda = [](core::JointPtr joint, const std::vector<double> force) -> bool {
        return joint->setJointGeneralizedForceTarget(force);
    };

    return Impl::setJointDataSerialized(this, forces, jointNames, lambda);
}

std::vector<double> Model::jointGeneralizedForceTargets(
    const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint) -> std::vector<double> {
        return joint->jointGeneralizedForceTarget();
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

// ======================
// Implementation Methods
// ======================

std::vector<double> Model::Impl::getJointDataSerialized(
    const Model* model,
    const std::vector<std::string>& jointNames,
    std::function<std::vector<double>(core::JointPtr)> getJointData)
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? model->jointNames() : jointNames;

    std::vector<double> data;
    data.reserve(jointSerialization.size());

    // data.reserve(model->dofs(jointSerialization));

    for (auto& joint : model->joints(jointSerialization)) {
        for (auto& value: getJointData(joint)) {
            data.push_back(value);
        }
    }

    return data;
}

bool Model::Impl::setJointDataSerialized(
    Model* model,
    const std::vector<double>& data,
    const std::vector<std::string>& jointNames,
    std::function<bool(core::JointPtr, const std::vector<double>)> setJointData)
{
    std::vector<std::string> jointSerialization;

    size_t expectedDOFs = 0;

    jointSerialization = jointNames.empty() ? model->jointNames() : jointNames;
    expectedDOFs = model->dofs(jointSerialization);

    if (data.size() != expectedDOFs) {
        LOG(ERROR) << "The size of value being set for each joint ";
        //            << "does not match the considered joint's DOFs."
        //            << std::endl;
        return false;
    }

    auto it = data.begin();

    std::vector<double> values(1);
    // std::vector<double> values;
    // values.reserve(1);

    for (auto& joint : model->joints(jointNames)) {

        // std::vector<double> values;
        // values.reserve(joint->dofs());
        //
        // for (size_t dof = 0; dof < joint->dofs(); ++dof)
        //   values.push_back(*it++);

        for (size_t dof = 0; dof < joint->dofs(); ++dof)
          values[0]=*it++;

        if (!setJointData(joint, values)) {
            LOG(ERROR) << "Failed to set force of joint '" << joint->name();
            //            << "'" << std::endl;
            return false;
        }
    }
    assert(it == data.end());
    return true;
}

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
