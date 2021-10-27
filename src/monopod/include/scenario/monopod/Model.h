
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
     * Get the joint generalized forces.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The serialization of joint forces. The vector has as many
     * elements as DoFs of the considered joints.
     */
    virtual std::vector<double> jointGeneralizedForces( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the joint limits of the model.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The joint limits of the model. The vectors of the limit
     * object have as many elements as DoFs of the considered joints.
     */
    virtual JointLimit jointLimits( //
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
     * Get the links of the model.
     *
     * @param linkNames Optional vector of considered links. By default,
     * ``Model::linkNames`` is used.
     * @return A vector of pointers to the link objects.
     */
    virtual std::vector<LinkPtr> links( //
        const std::vector<std::string>& linkNames = {}) const = 0;

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
     * Set the position targets of the joints.
     *
     * @param positions The vector with the joint position targets. It must
     * have as many elements as the considered joint DoFs.
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return True for success, false otherwise.
     */
    virtual bool setJointPositionTargets( //
        const std::vector<double>& positions,
        const std::vector<std::string>& jointNames = {}) = 0;

    /**
     * Set the velocity targets of the joints.
     *
     * @param velocities The vector with the joint velocity targets. It must
     * have as many elements as the considered joint DoFs.
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return True for success, false otherwise.
     */
    virtual bool setJointVelocityTargets( //
        const std::vector<double>& velocities,
        const std::vector<std::string>& jointNames = {}) = 0;

    /**
     * Set the acceleration targets of the joints.
     *
     * @param accelerations The vector with the joint acceleration targets.
     * It must have as many elements as the considered joint DoFs.
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return True for success, false otherwise.
     */
    virtual bool setJointAccelerationTargets( //
        const std::vector<double>& accelerations,
        const std::vector<std::string>& jointNames = {}) = 0;

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
     * Get the position targets of the joints.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return The position targets of the joints.
     */
    virtual std::vector<double> jointPositionTargets( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the velocity targets of the joints.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return The velocity targets of the joints.
     */
    virtual std::vector<double> jointVelocityTargets( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the acceleration targets of the joints.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return The acceleration targets of the joints.
     */
    virtual std::vector<double> jointAccelerationTargets( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the generalized force targets of the joints.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return The generalized force targets of the joints.
     */
    virtual std::vector<double> jointGeneralizedForceTargets( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    // =========
    // Base Link
    // =========

    /**
     * Get the name of the model's base frame.
     *
     * By default, the base frame is typically the root of the kinematic tree of
     * the model.
     *
     * @return The name of the model's base frame.
     */
    virtual std::string baseFrame() const = 0;

    /**
     * Get the position of the base link.
     *
     * @return The position of the base link in world coordinates.
     */
    virtual std::array<double, 3> basePosition() const = 0;

    /**
     * Get the orientation of the base link.
     *
     * @return The wxyz quaternion defining the orientation of the base link wrt
     * the world frame.
     */
    virtual std::array<double, 4> baseOrientation() const = 0;

    /**
     * Get the linear body velocity of the base link.
     *
     * @todo Add link to the velocity representation documentation page.
     *
     * @return The linear body velocity of the base link.
     */
    virtual std::array<double, 3> baseBodyLinearVelocity() const = 0;

    /**
     * Get the angular body velocity of the base link.
     *
     * @todo Add link to the velocity representation documentation page.
     *
     * @return The angular body velocity of the base link.
     */
    virtual std::array<double, 3> baseBodyAngularVelocity() const = 0;

    /**
     * Get the linear mixed velocity of the base link.
     *
     * @todo Add link to the velocity representation documentation page.
     *
     * @return The linear mixed velocity of the base link.
     */
    virtual std::array<double, 3> baseWorldLinearVelocity() const = 0;

    /**
     * Get the angular mixed velocity of the base link.
     *
     * @todo Add link to the velocity representation documentation page.
     *
     * @return The angular mixed velocity of the base link.
     */
    virtual std::array<double, 3> baseWorldAngularVelocity() const = 0;

    // =================
    // Base Link Targets
    // =================

    /**
     * Set the pose target of the base link.
     *
     * @param position The position target of the base link in world
     * coordinates.
     * @param orientation The wxyz quaternion defining the orientation target of
     * the base link wrt the world frame.
     * @return True for success, false otherwise.
     */
    virtual bool
    setBasePoseTarget(const std::array<double, 3>& position,
                      const std::array<double, 4>& orientation) = 0;

    /**
     * Set the position target of the base link.
     *
     * @param position The position target of the base link in world
     * coordinates.
     * @return True for success, false otherwise.
     */
    virtual bool
    setBasePositionTarget(const std::array<double, 3>& position) = 0;

    /**
     * Set the orientation target of the base link.
     *
     * @param orientation The wxyz quaternion defining the orientation target of
     * the base link wrt the world frame.
     * @return True for success, false otherwise.
     */
    virtual bool
    setBaseOrientationTarget(const std::array<double, 4>& orientation) = 0;

    /**
     * Set the mixed velocity target of the base link.
     *
     * @param linear The mixed linear velocity target of the base link.
     * @param angular The mixed angular velocity target of the base link.
     * @return True for success, false otherwise.
     */
    virtual bool
    setBaseWorldVelocityTarget(const std::array<double, 3>& linear,
                               const std::array<double, 3>& angular) = 0;

    /**
     * Set the mixed linear velocity target of the base link.
     *
     * @param linear The mixed linear velocity target of the base link.
     * @return True for success, false otherwise.
     */
    virtual bool
    setBaseWorldLinearVelocityTarget(const std::array<double, 3>& linear) = 0;

    /**
     * Set the mixed angular velocity target of the base link.
     *
     * @param angular The mixed angular velocity target of the base link.
     * @return True for success, false otherwise.
     */
    virtual bool setBaseWorldAngularVelocityTarget( //
        const std::array<double, 3>& angular) = 0;

    /**
     * Set the mixed linear acceleration target of the base link.
     *
     * @param linear The mixed linear acceleration target of the base link.
     * @return True for success, false otherwise.
     */
    virtual bool setBaseWorldLinearAccelerationTarget( //
        const std::array<double, 3>& linear) = 0;

    /**
     * Set the mixed angular acceleration target of the base link.
     *
     * @param angular The mixed angular acceleration target of the base link.
     * @return True for success, false otherwise.
     */
    virtual bool setBaseWorldAngularAccelerationTarget( //
        const std::array<double, 3>& angular) = 0;

    /**
     * Get the position target of the base link.
     *
     * @return The position target of the base link.
     */
    virtual std::array<double, 3> basePositionTarget() const = 0;

    /**
     * Get the orientation target of the base link.
     *
     * @return The quaternion defining the orientation target of the base link.
     */
    virtual std::array<double, 4> baseOrientationTarget() const = 0;

    /**
     * Get the mixed linear velocity target of the base link.
     *
     * @return The mixed linear velocity target of the base link.
     */
    virtual std::array<double, 3> baseWorldLinearVelocityTarget() const = 0;

    /**
     * Get the mixed angular velocity target of the base link.
     *
     * @return The mixed angular velocity target of the base link.
     */
    virtual std::array<double, 3> baseWorldAngularVelocityTarget() const = 0;

    /**
     * Get the mixed linear acceleration target of the base link.
     *
     * @return The mixed linear acceleration target of the base link.
     */
    virtual std::array<double, 3> baseWorldLinearAccelerationTarget() const = 0;

    /**
     * Get the mixed angular acceleration target of the base link.
     *
     * @return The mixed angular acceleration target of the base link.
     */
    virtual std::array<double, 3>
    baseWorldAngularAccelerationTarget() const = 0;

  private:
      class Impl;
      std::unique_ptr<Impl> pImpl;
  };
#endif // SCENARIO_MONOPOD_MODEL_H



// // ==========
// // Model Core
// // ==========
//
// bool valid() const override;
//
// size_t dofs(const std::vector<std::string>& jointNames = {}) const override;
//
// std::string name() const override;
//
// size_t nrOfLinks() const override;
//
// size_t nrOfJoints() const override;
//
// double
// totalMass(const std::vector<std::string>& linkNames = {}) const override;
//
// core::LinkPtr getLink(const std::string& linkName) const override;
//
// core::JointPtr getJoint(const std::string& jointName) const override;
//
// std::vector<std::string>
// linkNames(const bool scoped = false) const override;
//
// std::vector<std::string>
// jointNames(const bool scoped = false) const override;
//
// double controllerPeriod() const override;
//
// bool setControllerPeriod(const double period) override;
//
// bool enableHistoryOfAppliedJointForces(
//   const bool enable = true,
//   const size_t maxHistorySizePerJoint = 100,
//   const std::vector<std::string>& jointNames = {}) override;
//
// bool historyOfAppliedJointForcesEnabled(
//   const std::vector<std::string>& jointNames = {}) const override;
//
// std::vector<double> historyOfAppliedJointForces(
//   const std::vector<std::string>& jointNames = {}) const override;
//
// // ========
// // Contacts
// // ========
//
// bool contactsEnabled() const override;
//
// bool enableContacts(const bool enable = true) override;
//
// std::vector<std::string> linksInContact() const override;
//
// std::vector<core::Contact>
// contacts(const std::vector<std::string>& linkNames = {}) const override;
//
// // ==================
// // Vectorized Methods
// // ==================
//
// std::vector<double> jointPositions( //
//   const std::vector<std::string>& jointNames = {}) const override;
//
// std::vector<double> jointVelocities( //
//   const std::vector<std::string>& jointNames = {}) const override;
//
// std::vector<double> jointAccelerations( //
//   const std::vector<std::string>& jointNames = {}) const override;
//
// std::vector<double> jointGeneralizedForces( //
//   const std::vector<std::string>& jointNames = {}) const override;
//
// core::JointLimit jointLimits( //
//   const std::vector<std::string>& jointNames = {}) const override;
//
// bool setJointControlMode( //
//   const core::JointControlMode mode,
//   const std::vector<std::string>& jointNames = {}) override;
//
// std::vector<core::LinkPtr> links( //
//   const std::vector<std::string>& linkNames = {}) const override;
//
// std::vector<core::JointPtr> joints( //
//   const std::vector<std::string>& jointNames = {}) const override;
//
// // =========================
// // Vectorized Target Methods
// // =========================
//
// bool setJointPositionTargets( //
//   const std::vector<double>& positions,
//   const std::vector<std::string>& jointNames = {}) override;
//
// bool setJointVelocityTargets( //
//   const std::vector<double>& velocities,
//   const std::vector<std::string>& jointNames = {}) override;
//
// bool setJointAccelerationTargets( //
//   const std::vector<double>& accelerations,
//   const std::vector<std::string>& jointNames = {}) override;
//
// bool setJointGeneralizedForceTargets( //
//   const std::vector<double>& forces,
//   const std::vector<std::string>& jointNames = {}) override;
//
// std::vector<double> jointPositionTargets( //
//   const std::vector<std::string>& jointNames = {}) const override;
//
// std::vector<double> jointVelocityTargets( //
//   const std::vector<std::string>& jointNames = {}) const override;
//
// std::vector<double> jointAccelerationTargets( //
//   const std::vector<std::string>& jointNames = {}) const override;
//
// std::vector<double> jointGeneralizedForceTargets( //
//   const std::vector<std::string>& jointNames = {}) const override;
//
// // =========
// // Base Link
// // =========
//
// std::string baseFrame() const override;
//
// std::array<double, 3> basePosition() const override;
//
// std::array<double, 4> baseOrientation() const override;
//
// std::array<double, 3> baseBodyLinearVelocity() const override;
//
// std::array<double, 3> baseBodyAngularVelocity() const override;
//
// std::array<double, 3> baseWorldLinearVelocity() const override;
//
// std::array<double, 3> baseWorldAngularVelocity() const override;
//
// // =================
// // Base Link Targets
// // =================
//
// bool setBasePoseTarget( //
//   const std::array<double, 3>& position,
//   const std::array<double, 4>& orientation) override;
//
// bool setBasePositionTarget( //
//   const std::array<double, 3>& position) override;
//
// bool setBaseOrientationTarget( //
//   const std::array<double, 4>& orientation) override;
//
// bool setBaseWorldVelocityTarget( //
//   const std::array<double, 3>& linear,
//   const std::array<double, 3>& angular) override;
//
// bool setBaseWorldLinearVelocityTarget( //
//   const std::array<double, 3>& linear) override;
//
// bool setBaseWorldAngularVelocityTarget( //
//   const std::array<double, 3>& angular) override;
//
// bool setBaseWorldLinearAccelerationTarget( //
//   const std::array<double, 3>& linear) override;
//
// bool setBaseWorldAngularAccelerationTarget( //
//   const std::array<double, 3>& angular) override;
//
// std::array<double, 3> basePositionTarget() const override;
//
// std::array<double, 4> baseOrientationTarget() const override;
//
// std::array<double, 3> baseWorldLinearVelocityTarget() const override;
//
// std::array<double, 3> baseWorldAngularVelocityTarget() const override;
//
// std::array<double, 3> baseWorldLinearAccelerationTarget() const override;
//
std::array<double, 3> baseWorldAngularAccelerationTarget() const override;
