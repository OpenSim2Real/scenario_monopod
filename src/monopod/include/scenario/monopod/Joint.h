
#ifndef SCENARIO_MONOPOD_JOINT_H
#define SCENARIO_MONOPOD_JOINT_H

#include "scenario/core/Joint.h"

#include <memory>
#include <string>
#include <vector>

namespace scenario::monopod {
    class Joint;
} // namespace scenario::monopod

class scenario::monopod::Joint final
    : public scenario::core::Joint
{
public:
    Joint();
    virtual ~Joint();

        uint64_t id() const override;

        bool initialize(const string _name) override;
        
        /**
         * Check if the joint is valid.
         *
         * @return True if the model is valid, false otherwise.
         */
        virtual bool valid() const = 0;

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
        JointType type() const override;

        /**
         * Set the joint control mode.
         *
         * @param mode The desired control mode.
         * @return True for success, false otherwise.
         */
        bool setControlMode(const JointControlMode mode) override;

        /**
         * get the joint control mode.
         *
         * @return the joint control mode
         */

        scenario::core::JointControlMode Joint::controlMode() override;

        /**
         * Get the PID parameters of the joint.
         *
         * If no PID parameters have been set, the default parameters are
         * returned.
         *
         * @return The joint PID parameters.
         */

        PID pid() const override;

        /**
         * Set the PID parameters of the joint.
         *
         * @param pid The desired PID parameters.
         * @return True for success, false otherwise.
         */
        bool setPID(const PID& pid) override;

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
    };

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
