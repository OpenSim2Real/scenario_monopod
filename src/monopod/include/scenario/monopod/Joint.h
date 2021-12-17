
#ifndef SCENARIO_MONOPOD_JOINT_H
#define SCENARIO_MONOPOD_JOINT_H

#include "scenario/core/Joint.h"

#include <monopod_sdk/monopod.hpp>

#include <memory>
#include <string>
#include <vector>

namespace scenario::monopod {
    class Joint;
} // namespace scenario::monopod

// using namespace scenario::monopod;
class scenario::monopod::Joint final
      : public scenario::core::Joint
      , public std::enable_shared_from_this<scenario::monopod::Joint>
{
public:
    Joint();
    virtual ~Joint();

    uint64_t id() const;


    bool initialize(const std::string name,
                    const std::string parentModelName,
                    const std::shared_ptr<monopod_drivers::Monopod> &monopod_sdk);

  // bool initialize(const std::string name,
  //                 const std::string parentModelName);

    /**
     * Check if the joint is valid.
     *
     * @return True if the model is valid, false otherwise.
     */
    bool valid() const override;

    /**
     * Get the number of degrees of freedom of the joint.
     *
     * @return The number of DOFs of the joint.
     */
    size_t dofs() const override;

    /**
     * Get the name of the joint.
     *
     * @param scoped If true, the scoped name of the joint is returned.
     * @return The name of the joint.
     */
    std::string name(const bool scoped = false) const override;

    /**
     * Get the type of the joint.
     *
     * @return The type of the joint.
     */
    core::JointType type() const override;

    /**
     * Set the joint control mode.
     *
     * @param mode The desired control mode.
     * @return True for success, false otherwise.
     */
    bool setControlMode(const core::JointControlMode mode) override;

    /**
     * get the joint control mode.
     *
     * @return the joint control mode
     */

    core::JointControlMode controlMode() const override;

    /**
     * Get the PID parameters of the joint.
     *
     * If no PID parameters have been set, the default parameters are
     * returned.
     *
     * @return The joint PID parameters.
     */

    core::PID pid() const override;

    /**
     * Set the PID parameters of the joint.
     *
     * @param pid The desired PID parameters.
     * @return True for success, false otherwise.
     */
    bool setPID(const core::PID& pid) override;

    /**
     * Get the maximum generalized force that could be applied to the joint.
     *
     * @return The maximum generalized force of the joint.
     */
    std::vector<double> jointMaxGeneralizedForce() const override;

    /**
     * Set the maximum generalized force that can be applied to the joint.
     *
     * This limit can be used to clip the force applied by joint
     * controllers.
     *
     * @param maxForce A vector with the maximum generalized forces of the
     * joint DOFs.
     * @return True for success, false otherwise.
     */
    bool
    setJointMaxGeneralizedForce(const std::vector<double>& maxForce) override;

    /**
     * Get the position of the joint.
     *
     * @return The position of the joint.
     */
    std::vector<double> jointPosition() const override;

    /**
     * Get the velocity of the joint.
     *
     * @return The velocity of the joint.
     */
    std::vector<double> jointVelocity() const override;

    /**
     * Get the acceleration of the joint.
     *
     * @return The acceleration of the joint.
     */
    std::vector<double> jointAcceleration() const override;

    /**
     * Set the generalized force target of the joint.
     *
     * Note that if there's friction or other loss components, the real
     * joint force will differ.
     *
     * @param force A vector with the generalized force targets of the joint
     * DOFs.
     * @return True for success, false otherwise.
     */
    bool
    setJointGeneralizedForceTarget(const std::vector<double>& force) override;

    /**
     * Get the active generalized force target.
     *
     * @return The generalized force target of the joint.
     */
    std::vector<double> jointGeneralizedForceTarget() const override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // =============
    // Scenario core
    // =============

   double controllerPeriod() const override;
   bool historyOfAppliedJointForcesEnabled() const override;
   bool enableHistoryOfAppliedJointForces(const bool enable = true, const size_t maxHistorySize = 100) override;
   std::vector<double> historyOfAppliedJointForces() const override;
   double coulombFriction() const override;
   double viscousFriction() const override;
   // ==================
   // Single DOF methods
   // ==================
   core::Limit positionLimit(const size_t dof = 0) const override;
   core::Limit velocityLimit(const size_t dof = 0) const override;
   bool setVelocityLimit(const double maxVelocity, const size_t dof = 0) override;
   double maxGeneralizedForce(const size_t dof = 0) const override;
   bool setMaxGeneralizedForce(const double maxForce, const size_t dof = 0) override;
   double position(const size_t dof = 0) const override;
   double velocity(const size_t dof = 0) const override;
   double acceleration(const size_t dof = 0) const override;
   double generalizedForce(const size_t dof = 0) const override;
   bool setPositionTarget(const double position, const size_t dof = 0) override;
   bool setVelocityTarget(const double velocity, const size_t dof = 0) override;
   bool setAccelerationTarget(const double acceleration, const size_t dof = 0) override;
   bool setGeneralizedForceTarget(const double force, const size_t dof = 0) override;
   double positionTarget(const size_t dof = 0) const override;
   double velocityTarget(const size_t dof = 0) const override;
   double accelerationTarget(const size_t dof = 0) const override;
   double generalizedForceTarget(const size_t dof = 0) const override;
   // =================
   // Multi DOF methods
   // =================
   core::JointLimit jointPositionLimit() const override;
   core::JointLimit jointVelocityLimit() const override;
   bool setJointVelocityLimit(const std::vector<double>& maxVelocity) override;
   std::vector<double> jointGeneralizedForce() const override;
   bool setJointPositionTarget(const std::vector<double>& position) override;
   bool setJointVelocityTarget(const std::vector<double>& velocity) override;
   bool setJointAccelerationTarget(const std::vector<double>& acceleration) override;
   std::vector<double> jointPositionTarget() const override;
   std::vector<double> jointVelocityTarget() const override;
   std::vector<double> jointAccelerationTarget() const override;
};

// =============
// Scenario core
// =============
inline double scenario::monopod::Joint::controllerPeriod() const {exit(0);}
inline bool scenario::monopod::Joint::historyOfAppliedJointForcesEnabled() const {exit(0);}
inline bool scenario::monopod::Joint::enableHistoryOfAppliedJointForces( const bool enable, const size_t maxHistorySize) {exit(0);}
inline std::vector<double> scenario::monopod::Joint::historyOfAppliedJointForces() const {exit(0);}
inline double scenario::monopod::Joint::coulombFriction() const {exit(0);}
inline double scenario::monopod::Joint::viscousFriction() const {exit(0);}
// ==================
// Single DOF methods
// ==================
inline scenario::core::Limit scenario::monopod::Joint::positionLimit(const size_t dof) const {exit(0);}
inline scenario::core::Limit scenario::monopod::Joint::velocityLimit(const size_t dof) const {exit(0);}
inline bool scenario::monopod::Joint::setVelocityLimit(const double maxVelocity, const size_t dof) {exit(0);}
inline double scenario::monopod::Joint::maxGeneralizedForce(const size_t dof) const {exit(0);}
inline bool scenario::monopod::Joint::setMaxGeneralizedForce(const double maxForce, const size_t dof) {exit(0);}
inline double scenario::monopod::Joint::position(const size_t dof) const {exit(0);}
inline double scenario::monopod::Joint::velocity(const size_t dof) const {exit(0);}
inline double scenario::monopod::Joint::acceleration(const size_t dof) const {exit(0);}
inline double scenario::monopod::Joint::generalizedForce(const size_t dof) const {exit(0);}
inline bool scenario::monopod::Joint::setPositionTarget(const double position, const size_t dof) {exit(0);}
inline bool scenario::monopod::Joint::setVelocityTarget(const double velocity, const size_t dof) {exit(0);}
inline bool scenario::monopod::Joint::setAccelerationTarget(const double acceleration, const size_t dof) {exit(0);}
inline bool scenario::monopod::Joint::setGeneralizedForceTarget(const double force, const size_t dof) {exit(0);}
inline double scenario::monopod::Joint::positionTarget(const size_t dof) const {exit(0);}
inline double scenario::monopod::Joint::velocityTarget(const size_t dof) const {exit(0);}
inline double scenario::monopod::Joint::accelerationTarget(const size_t dof) const {exit(0);}
inline double scenario::monopod::Joint::generalizedForceTarget(const size_t dof) const {exit(0);}
// =================
// Multi DOF methods
// =================
inline scenario::core::JointLimit scenario::monopod::Joint::jointPositionLimit() const {exit(0);}
inline scenario::core::JointLimit scenario::monopod::Joint::jointVelocityLimit() const {exit(0);}
inline bool scenario::monopod::Joint::setJointVelocityLimit(const std::vector<double>& maxVelocity) {exit(0);}
inline std::vector<double> scenario::monopod::Joint::jointGeneralizedForce() const {exit(0);}
inline bool scenario::monopod::Joint::setJointVelocityTarget(const std::vector<double>& velocity) {exit(0);}
inline bool scenario::monopod::Joint::setJointPositionTarget(const std::vector<double>& position) {exit(0);}
inline bool scenario::monopod::Joint::setJointAccelerationTarget(const std::vector<double>& acceleration) {exit(0);}
inline std::vector<double> scenario::monopod::Joint::jointPositionTarget() const {exit(0);}
inline std::vector<double> scenario::monopod::Joint::jointVelocityTarget() const {exit(0);}
inline std::vector<double> scenario::monopod::Joint::jointAccelerationTarget() const {exit(0);}

#endif // SCENARIO_MONOPOD_JOINT_H

// /**
//  * Get the position limits of the joint.
//  *
//  * @return The position limits of the joint.
//  */
// virtual JointLimit jointPositionLimit() const = 0;
//
// /**
//  * Get the velocity limits of the joint.
//  *
//  * @return The velocity limits of the joint.
//  */
// virtual JointLimit jointVelocityLimit() const = 0;
//
// /**
//  * Set the maximum velocity of the joint.
//  *
//  * This limit can be used to clip the velocity applied by joint
//  * controllers.
//  *
//  * @param maxVelocity A vector with the maximum velocity of the joint DOFs.
//  * @return True for success, false otherwise.
//  */
// virtual bool
// setJointVelocityLimit(const std::vector<double>& maxVelocity) = 0;
