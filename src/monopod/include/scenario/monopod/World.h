
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
      , public std::enable_shared_from_this<scenario::monopod::World>
{
public:
    World();
    virtual ~World();

    uint64_t id() const;

    /**
     * Check if the world is valid.
     *
     * @return True if the world is valid, false otherwise.
     */
    bool valid() const override;

    /**
     * Get the name of the world.
     *
     * @return The name of the world.
     */
    std::string name() const override;

    /**
     * Get the name of the models that are part of the world.
     *
     * @return The list of model names.
     */
    std::vector<std::string> modelNames() const override;

    /**
     * Get a model part of the world.
     *
     * @param modelName The name of the model to get.
     * @return The model if it is part of the world, ``nullptr`` otherwise.
     */
    scenario::core::ModelPtr getModel(const std::string& modelName) const override;

    /**
     * Get the models of the world.
     *
     * @param modelNames Optional vector of considered models. By default,
     * ``World::modelNames`` is used.
     * @return A vector of pointers to the model objects.
     */
    std::vector<scenario::core::ModelPtr> models( //
        const std::vector<std::string>& modelNames = {}) const override;
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
    // ==========
    // World Core
    // ==========

    double time() const override;
    std::array<double, 3> gravity() const override;
};

inline double scenario::monopod::World::time() const {exit(0);}
inline std::array<double, 3> scenario::monopod::World::gravity() const {exit(0);}

#endif // SCENARIO_MONOPOD_WORLD_H
