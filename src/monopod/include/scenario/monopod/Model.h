
#ifndef SCENARIO_MONOPOD_MODEL_H
#define SCENARIO_MONOPOD_MODEL_H

#include "scenario/core/Model.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario::gazebo {
    class Model;
} // namespace scenario::monopod

class scenario::gazebo::Model final
    : public scenario::core::Model
{
public:
    Model();
    virtual ~Model();

    /**
     * Check if the model is valid.
     *
     * @return True if the model is valid, false otherwise.
     */
    virtual bool valid() const = 0;

    /**
     * Get the name of the model.
     *
     * @return The name of the model.
     */
    virtual std::string name() const = 0;

    /**
     * Get a joint belonging to the model.
     *
     * @param jointName The name of the joint.
     * @throw std::runtime_error if the joint does not exist.
     * @return The desired joint.
     */
    virtual JointPtr getJoint(const std::string& jointName) const = 0;

    /**
     * Get the name of all the model's joints.
     *
     * @param scoped Scope the joint names with the model name,
     * (e.g. ``mymodel::joint1``).
     * @return The list of joint names.
     */
    virtual std::vector<std::string>
    jointNames(const bool scoped = false) const = 0;

    // ==================
    // Vectorized Methods
    // ==================

    /**
     * Get the joint positions.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The serialization of joint positions. The vector has as many
     * elements as DoFs of the considered joints.
     */
    virtual std::vector<double> jointPositions( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the joint velocities.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The serialization of joint velocities. The vector has as many
     * elements as DoFs of the considered joints.
     */
    virtual std::vector<double> jointVelocities( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the joint accelerations.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The serialization of joint accelerations. The vector has as many
     * elements as DoFs of the considered joints.
     */
    virtual std::vector<double> jointAccelerations( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Set the control mode of model joints.
     *
     * @param mode The desired joint control mode.
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return True for success, false otherwise.
     */
    virtual bool
    setJointControlMode(const JointControlMode mode,
                        const std::vector<std::string>& jointNames = {}) = 0;

    /**
     * Get the joints of the model.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return A vector of pointers to the joint objects.
     */
    virtual std::vector<JointPtr> joints( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    // =========================
    // Vectorized Target Methods
    // =========================

    /**
     * Set the generalized force targets of the joints.
     *
     * @param forces The vector with the joint generalized force targets. It
     * must have as many elements as the considered joint DoFs.
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return True for success, false otherwise.
     */
    virtual bool setJointGeneralizedForceTargets( //
        const std::vector<double>& forces,
        const std::vector<std::string>& jointNames = {}) = 0;

    /**
     * Get the generalized force targets of the joints.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return The generalized force targets of the joints.
     */
    virtual std::vector<double> jointGeneralizedForceTargets( //
        const std::vector<std::string>& jointNames = {}) const = 0;

  private:
      class Impl;
      std::unique_ptr<Impl> pImpl;
  };
#endif // SCENARIO_MONOPOD_MODEL_H


// /**
//  * Get the joint limits of the model.
//  *
//  * @param jointNames Optional vector of considered joints that also
//  * defines the joint serialization. By default, ``Model::jointNames`` is
//  * used.
//  * @return The joint limits of the model. The vectors of the limit
//  * object have as many elements as DoFs of the considered joints.
//  */
// virtual JointLimit jointLimits( //
//     const std::vector<std::string>& jointNames = {}) const = 0;
