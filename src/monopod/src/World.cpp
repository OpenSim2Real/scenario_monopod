/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This project is dual licensed under LGPL v2.1+ or Apache License.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "scenario/monopod/World.h"
#include "scenario/monopod/Log.h"
#include "scenario/monopod/Model.h"

#include <algorithm>
#include <cassert>
#include <functional>
#include <unordered_map>

using namespace scenario::gazebo;

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

public:
};


World::World()
    : pImpl{std::make_unique<Impl>()}
{}

World::~World() = default;

uint64_t World::id() const
{
    return std::hash<std::string>{}(this->name());
}

bool World::initialize()
{
    pImpl->buffers.worldName  = "real_world";
    // initialize the model.
    pImpl->buffers.modelNames.clear();
    pImpl->buffers.modelNames.push_back("monopod");
    return true;
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
    if (pImpl->models.find(modelName) != pImpl->models.end()) {
        assert(pImpl->models.at(modelName));
        return pImpl->models.at(modelName);
    }
    
    // Create and initialize the model if no model was found
    auto model = std::make_shared<scenario::gazebo::Model>();
    model->initialize(modelEntity, m_ecm, m_eventManager);

    pImpl->models[modelName] = model;
    return pImpl->models[modelName];
}
