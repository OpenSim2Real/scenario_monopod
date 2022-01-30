
#ifndef SCENARIO_MONOPOD_MODEL_H
#define SCENARIO_MONOPOD_MODEL_H

#include "scenario/core/Model.h"
#include <monopod_sdk/monopod.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario::monopod {
class Model;
} // namespace scenario::monopod

class scenario::monopod::Model final
    : public scenario::core::Model,
      public std::enable_shared_from_this<scenario::monopod::Model> {
public:
  Model();
  virtual ~Model();

  /**
   * @brief A useful shortcut for using cpp scenario code.
   */
  typedef monopod_drivers::Mode Mode;

  /**
   * @brief Initialize can_bus connections to encoder board and motor board.
   *
   * @param monopod_mode defines the task mode of the monopod. Can also specify
   * individual boards.
   */
  bool initialize(const monopod_drivers::Mode &mode) const;

  /**
   * @brief Calibrate the Encoders.
   *
   * @param hip_home_offset_rad hip offset from found encoder index 0 (rad)
   * @param knee_home_offset_rad knee offset from found encoder index 0 (rad)
   */
  void calibrate(const double &hip_home_offset_rad = 0,
                 const double &knee_home_offset_rad = 0);

  /**
   * @brief If the joint module is not valid (safemode after limit reached) the
   * joint will be reset into a valid state. This means the joint must be set
   * back into the valid state first otherwise it will trigger the limits again.
   */
  void reset();

  /**
   * @brief print status messages of robot.
   */
  void print_status();

  /**
   * Check if the model is valid.
   *
   * @return True if the model is valid, false otherwise.
   */
  bool valid() const override;

  /**
   * Get the name of the model.
   *
   * @return The name of the model.
   */
  std::string name() const override;

  /**
   * Get the joints DOF
   *
   * @param jointNames Optional vector of considered joints that also
   * defines the joint serialization. By default, ``Model::jointNames`` is
   * used.
   * @return The sum of serialization of joint DOFs. The sum is the number of
   * DoFs of all the considered joints.
   */
  size_t dofs(const std::vector<std::string> &jointNames = {}) const override;

  /**
   * Get a joint belonging to the model.
   *
   * @param jointName The name of the joint.
   * @throw std::runtime_error if the joint does not exist.
   * @return The desired joint.
   */
  core::JointPtr getJoint(const std::string &jointName) const override;

  /**
   * Get the name of all the model's joints.
   *
   * @param scoped Scope the joint names with the model name,
   * (e.g. ``mymodel::joint1``).
   * @return The list of joint names.
   */
  std::vector<std::string> jointNames(const bool scoped = false) const override;

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
  std::vector<double> jointPositions( //
      const std::vector<std::string> &jointNames = {}) const override;

  /**
   * Get the joint velocities.
   *
   * @param jointNames Optional vector of considered joints that also
   * defines the joint serialization. By default, ``Model::jointNames`` is
   * used.
   * @return The serialization of joint velocities. The vector has as many
   * elements as DoFs of the considered joints.
   */
  std::vector<double> jointVelocities( //
      const std::vector<std::string> &jointNames = {}) const override;

  /**
   * Get the joint accelerations.
   *
   * @param jointNames Optional vector of considered joints that also
   * defines the joint serialization. By default, ``Model::jointNames`` is
   * used.
   * @return The serialization of joint accelerations. The vector has as many
   * elements as DoFs of the considered joints.
   */
  std::vector<double> jointAccelerations( //
      const std::vector<std::string> &jointNames = {}) const override;

  /**
   * Set the control mode of model joints.
   *
   * @param mode The desired joint control mode.
   * @param jointNames Optional vector of considered joints that also
   * defines the joint serialization. By default, ``Model::jointNames`` is
   * used.
   * @return True for success, false otherwise.
   */
  bool
  setJointControlMode(const core::JointControlMode mode,
                      const std::vector<std::string> &jointNames = {}) override;

  /**
   * Get the joints of the model.
   *
   * @param jointNames Optional vector of considered joints. By default,
   * ``Model::jointNames`` is used.
   * @return A vector of pointers to the joint objects.
   */
  std::vector<core::JointPtr> joints( //
      const std::vector<std::string> &jointNames = {}) const override;

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
  bool setJointGeneralizedForceTargets( //
      const std::vector<double> &forces,
      const std::vector<std::string> &jointNames = {}) override;

  /**
   * Get the generalized force targets of the joints.
   *
   * @param jointNames Optional vector of considered joints. By default,
   * ``Model::jointNames`` is used.
   * @return The generalized force targets of the joints.
   */
  std::vector<double> jointGeneralizedForceTargets( //
      const std::vector<std::string> &jointNames = {}) const override;

  bool setControllerPeriod(const double period) override;

private:
  class Impl;
  std::unique_ptr<Impl> pImpl;

  // ==========
  // Model Core
  // ==========

  size_t nrOfLinks() const override;
  size_t nrOfJoints() const override;
  double
  totalMass(const std::vector<std::string> &linkNames = {}) const override;
  core::LinkPtr getLink(const std::string &linkName) const override;
  std::vector<std::string> linkNames(const bool scoped = false) const override;
  double controllerPeriod() const override;
  bool enableHistoryOfAppliedJointForces(
      const bool enable = true, const size_t maxHistorySizePerJoint = 100,
      const std::vector<std::string> &jointNames = {}) override;
  bool historyOfAppliedJointForcesEnabled(
      const std::vector<std::string> &jointNames = {}) const override;
  std::vector<double> historyOfAppliedJointForces(
      const std::vector<std::string> &jointNames = {}) const override;
  // ========
  // Contacts
  // ========
  bool contactsEnabled() const override;
  bool enableContacts(const bool enable = true) override;
  std::vector<std::string> linksInContact() const override;
  std::vector<core::Contact>
  contacts(const std::vector<std::string> &linkNames = {}) const override;
  // ==================
  // Vectorized Methods
  // ==================
  std::vector<double> jointGeneralizedForces( //
      const std::vector<std::string> &jointNames = {}) const override;
  core::JointLimit jointLimits( //
      const std::vector<std::string> &jointNames = {}) const override;
  std::vector<core::LinkPtr> links( //
      const std::vector<std::string> &linkNames = {}) const override;
  // =========================
  // Vectorized Target Methods
  // =========================
  bool setJointPositionTargets( //
      const std::vector<double> &positions,
      const std::vector<std::string> &jointNames = {}) override;
  bool setJointVelocityTargets( //
      const std::vector<double> &velocities,
      const std::vector<std::string> &jointNames = {}) override;
  bool setJointAccelerationTargets( //
      const std::vector<double> &accelerations,
      const std::vector<std::string> &jointNames = {}) override;
  std::vector<double> jointPositionTargets( //
      const std::vector<std::string> &jointNames = {}) const override;
  std::vector<double> jointVelocityTargets( //
      const std::vector<std::string> &jointNames = {}) const override;
  std::vector<double> jointAccelerationTargets( //
      const std::vector<std::string> &jointNames = {}) const override;
  // =========
  // Base Link
  // =========
  std::string baseFrame() const override;
  std::array<double, 3> basePosition() const override;
  std::array<double, 4> baseOrientation() const override;
  std::array<double, 3> baseBodyLinearVelocity() const override;
  std::array<double, 3> baseBodyAngularVelocity() const override;
  std::array<double, 3> baseWorldLinearVelocity() const override;
  std::array<double, 3> baseWorldAngularVelocity() const override;
  // =================
  // Base Link Targets
  // =================
  bool setBasePoseTarget( //
      const std::array<double, 3> &position,
      const std::array<double, 4> &orientation) override;
  bool setBasePositionTarget( //
      const std::array<double, 3> &position) override;
  bool setBaseOrientationTarget( //
      const std::array<double, 4> &orientation) override;
  bool setBaseWorldVelocityTarget( //
      const std::array<double, 3> &linear,
      const std::array<double, 3> &angular) override;
  bool setBaseWorldLinearVelocityTarget( //
      const std::array<double, 3> &linear) override;
  bool setBaseWorldAngularVelocityTarget( //
      const std::array<double, 3> &angular) override;
  bool setBaseWorldLinearAccelerationTarget( //
      const std::array<double, 3> &linear) override;
  bool setBaseWorldAngularAccelerationTarget( //
      const std::array<double, 3> &angular) override;
  std::array<double, 3> basePositionTarget() const override;
  std::array<double, 4> baseOrientationTarget() const override;
  std::array<double, 3> baseWorldLinearVelocityTarget() const override;
  std::array<double, 3> baseWorldAngularVelocityTarget() const override;
  std::array<double, 3> baseWorldLinearAccelerationTarget() const override;
  std::array<double, 3> baseWorldAngularAccelerationTarget() const override;
};

// ==========
// place holder
// ==========
inline bool scenario::monopod::Model::setControllerPeriod(const double period) {
  return true;
} // set as a dummy.

// ==========
// Model Core
// ==========
inline size_t scenario::monopod::Model::nrOfLinks() const { exit(0); }
inline size_t scenario::monopod::Model::nrOfJoints() const { exit(0); }
inline double scenario::monopod::Model::totalMass(
    const std::vector<std::string> &linkNames) const {
  exit(0);
}
inline scenario::core::LinkPtr
scenario::monopod::Model::getLink(const std::string &linkName) const {
  exit(0);
}
inline std::vector<std::string>
scenario::monopod::Model::linkNames(const bool scoped) const {
  exit(0);
}
inline double scenario::monopod::Model::controllerPeriod() const { exit(0); }
inline bool scenario::monopod::Model::enableHistoryOfAppliedJointForces(
    const bool enable, const size_t maxHistorySizePerJoint,
    const std::vector<std::string> &jointNames) {
  exit(0);
}
inline bool scenario::monopod::Model::historyOfAppliedJointForcesEnabled(
    const std::vector<std::string> &jointNames) const {
  exit(0);
}
inline std::vector<double>
scenario::monopod::Model::historyOfAppliedJointForces(
    const std::vector<std::string> &jointNames) const {
  exit(0);
}
// ========
// Contacts
// ========
inline bool scenario::monopod::Model::contactsEnabled() const { exit(0); }
inline bool scenario::monopod::Model::enableContacts(const bool enable) {
  exit(0);
}
inline std::vector<std::string>
scenario::monopod::Model::linksInContact() const {
  exit(0);
}
inline std::vector<scenario::core::Contact> scenario::monopod::Model::contacts(
    const std::vector<std::string> &linkNames) const {
  exit(0);
}
// ==================
// Vectorized Methods
// ==================
inline std::vector<double> scenario::monopod::Model::jointGeneralizedForces( //
    const std::vector<std::string> &jointNames) const {
  exit(0);
}
inline scenario::core::JointLimit scenario::monopod::Model::jointLimits( //
    const std::vector<std::string> &jointNames) const {
  exit(0);
}
inline std::vector<scenario::core::LinkPtr> scenario::monopod::Model::links( //
    const std::vector<std::string> &linkNames) const {
  exit(0);
}
// =========================
// Vectorized Target Methods
// =========================
inline bool scenario::monopod::Model::setJointPositionTargets( //
    const std::vector<double> &positions,
    const std::vector<std::string> &jointNames) {
  exit(0);
}
inline bool scenario::monopod::Model::setJointVelocityTargets( //
    const std::vector<double> &velocities,
    const std::vector<std::string> &jointNames) {
  exit(0);
}
inline bool scenario::monopod::Model::setJointAccelerationTargets( //
    const std::vector<double> &accelerations,
    const std::vector<std::string> &jointNames) {
  exit(0);
}
inline std::vector<double> scenario::monopod::Model::jointPositionTargets( //
    const std::vector<std::string> &jointNames) const {
  exit(0);
}
inline std::vector<double> scenario::monopod::Model::jointVelocityTargets( //
    const std::vector<std::string> &jointNames) const {
  exit(0);
}
inline std::vector<double>
scenario::monopod::Model::jointAccelerationTargets( //
    const std::vector<std::string> &jointNames) const {
  exit(0);
}
// =========
// Base Link
// =========
inline std::string scenario::monopod::Model::baseFrame() const { exit(0); }
inline std::array<double, 3> scenario::monopod::Model::basePosition() const {
  exit(0);
}
inline std::array<double, 4> scenario::monopod::Model::baseOrientation() const {
  exit(0);
}
inline std::array<double, 3>
scenario::monopod::Model::baseBodyLinearVelocity() const {
  exit(0);
}
inline std::array<double, 3>
scenario::monopod::Model::baseBodyAngularVelocity() const {
  exit(0);
}
inline std::array<double, 3>
scenario::monopod::Model::baseWorldLinearVelocity() const {
  exit(0);
}
inline std::array<double, 3>
scenario::monopod::Model::baseWorldAngularVelocity() const {
  exit(0);
}
// =================
// Base Link Targets
// =================
inline bool scenario::monopod::Model::setBasePoseTarget( //
    const std::array<double, 3> &position,
    const std::array<double, 4> &orientation) {
  exit(0);
}
inline bool scenario::monopod::Model::setBasePositionTarget( //
    const std::array<double, 3> &position) {
  exit(0);
}
inline bool scenario::monopod::Model::setBaseOrientationTarget( //
    const std::array<double, 4> &orientation) {
  exit(0);
}
inline bool scenario::monopod::Model::setBaseWorldVelocityTarget( //
    const std::array<double, 3> &linear, const std::array<double, 3> &angular) {
  exit(0);
}
inline bool scenario::monopod::Model::setBaseWorldLinearVelocityTarget( //
    const std::array<double, 3> &linear) {
  exit(0);
}
inline bool scenario::monopod::Model::setBaseWorldAngularVelocityTarget( //
    const std::array<double, 3> &angular) {
  exit(0);
}
inline bool scenario::monopod::Model::setBaseWorldLinearAccelerationTarget( //
    const std::array<double, 3> &linear) {
  exit(0);
}
inline bool scenario::monopod::Model::setBaseWorldAngularAccelerationTarget( //
    const std::array<double, 3> &angular) {
  exit(0);
}
inline std::array<double, 3>
scenario::monopod::Model::basePositionTarget() const {
  exit(0);
}
inline std::array<double, 4>
scenario::monopod::Model::baseOrientationTarget() const {
  exit(0);
}
inline std::array<double, 3>
scenario::monopod::Model::baseWorldLinearVelocityTarget() const {
  exit(0);
}
inline std::array<double, 3>
scenario::monopod::Model::baseWorldAngularVelocityTarget() const {
  exit(0);
}
inline std::array<double, 3>
scenario::monopod::Model::baseWorldLinearAccelerationTarget() const {
  exit(0);
}
inline std::array<double, 3>
scenario::monopod::Model::baseWorldAngularAccelerationTarget() const {
  exit(0);
}

#endif // SCENARIO_MONOPOD_MODEL_H
