#include "scenario/monopod/World.h"
#include "scenario/monopod/Model.h"
#include "scenario/monopod/Joint.h"

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <functional>
#include <unordered_map>

#include "scenario/monopod/easylogging++.h"

INITIALIZE_EASYLOGGINGPP

using namespace scenario::monopod;

class World::Impl
{
public:
    using ModelName = std::string;
    std::unordered_map<ModelName, core::ModelPtr> models;

    struct
    {
        std::vector<std::string> modelNames;
        std::string worldName;
    } buffers;
};

World::World()
    : pImpl{std::make_unique<Impl>()}
{
    // Set up the names for world and model.
    pImpl->buffers.worldName  = "real_world";
    std::string modelName = "monopod";
    // initialize the model.
    pImpl->buffers.modelNames.clear();
    pImpl->buffers.modelNames.push_back(modelName);

    // Initialize our model class
    scenario::core::ModelPtr model = std::make_shared<scenario::monopod::Model>();
    pImpl->models[modelName] = model;
}

World::~World() = default;

uint64_t World::id() const
{
    return std::hash<std::string>{}(this->name());
}

bool World::valid() const
{
    bool ok = true;
    for (auto& model : this->models(this->modelNames())) {
        ok = ok && model->valid();
    }
    return ok;
}

std::vector<scenario::core::ModelPtr> World::models(const std::vector<std::string>& modelNames) const
{
    const std::vector<std::string>& modelSerialization =
        modelNames.empty() ? this->modelNames() : modelNames;

    std::vector<scenario::core::ModelPtr> models;

    for (const auto& modelName : modelSerialization) {
        models.push_back(this->getModel(modelName));
    }

    return models;
}

std::string World::name() const
{
    return pImpl->buffers.worldName;
}

std::vector<std::string> World::modelNames() const
{
    return pImpl->buffers.modelNames;
}

scenario::core::ModelPtr World::getModel(const std::string& modelName) const
{
    // Get the model with the name
    if (pImpl->models.find(modelName) != pImpl->models.end()) {
        assert(pImpl->models.at(modelName));
        return pImpl->models.at(modelName);
    }

    std::string str = " ";
    for(auto& name: pImpl->buffers.modelNames)
        str = str + " " + name;

    LOG(ERROR) << "Model name does not exist in world. Available models are: " + str;
    throw std::invalid_argument( "Model name does not exist in world. Available models are: " + str);
}
