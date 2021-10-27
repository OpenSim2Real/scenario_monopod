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
     * Get the name of the world.
     *
     * @return The name of the world.
     */
    virtual std::string name() const = 0;

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
