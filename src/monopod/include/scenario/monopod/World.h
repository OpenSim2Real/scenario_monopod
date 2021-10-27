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

#ifndef SCENARIO_MONOPOD_WORLD_H
#define SCENARIO_MONOPOD_WORLD_H

#include "scenario/core/World.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario::monopod {
    class World;
} // namespace scenario::monopod

class scenario::monopod::World final
    : public scenario::core::World
{
public:
    World();
    virtual ~World();

    /**
     * Check if the world is valid.
     *
     * @return True if the world is valid, false otherwise.
     */
    virtual bool valid() const = 0;

    /**
     * Get the simulated time.
     *
     * @note A physics plugin need to be part of the simulation
     * in order to make the time flow.
     *
     * @return The simulated time.
     */
    virtual double time() const = 0;

    /**
     * Get the name of the world.
     *
     * @return The name of the world.
     */
    virtual std::string name() const = 0;

    /**
     * Get the gravity vector.
     * @return The gravity vector.
     */
    virtual std::array<double, 3> gravity() const = 0;

    /**
     * Get the name of the models that are part of the world.
     *
     * @return The list of model names.
     */
    virtual std::vector<std::string> modelNames() const = 0;

    /**
     * Get a model part of the world.
     *
     * @param modelName The name of the model to get.
     * @return The model if it is part of the world, ``nullptr`` otherwise.
     */
    virtual ModelPtr getModel(const std::string& modelName) const = 0;

    /**
     * Get the models of the world.
     *
     * @param modelNames Optional vector of considered models. By default,
     * ``World::modelNames`` is used.
     * @return A vector of pointers to the model objects.
     */
    virtual std::vector<ModelPtr> models( //
        const std::vector<std::string>& modelNames = {}) const = 0;
};

#endif // SCENARIO_MONOPOD_WORLD_H

// // ==========
// // World Core
// // ==========
//
// bool valid() const override;
//
// double time() const override;
//
// std::string name() const override;
//
// std::array<double, 3> gravity() const override;
//
// std::vector<std::string> modelNames() const override;
//
// scenario::core::ModelPtr
// getModel(const std::string& modelName) const override;
//
// std::vector<core::ModelPtr> models( //
//     const std::vector<std::string>& modelNames = {}) const override;
