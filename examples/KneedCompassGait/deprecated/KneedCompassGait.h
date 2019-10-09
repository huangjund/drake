#ifndef DRAKE_KNEEDCOMPASSGAIT_H
#define DRAKE_KNEEDCOMPASSGAIT_H

#pragma once

#include <memory>
#include <vector>

#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_ContinuousState.h"
#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_Params.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/witness_function.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/math/rotation_matrix.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace examples {
namespace KneedCompassGait {

/// Inputs:            HipTorque, LeftKneeTorque, RightKneeTorque
/// Continuous States: angle_stance_knee, angle_swing_knee,
///                    angle_hip, angle_thigh
///                    angledot_stance_knee, angledot_swing_knee,
///                    angledot_hip, angledot_thigh
/// Discrete State:    stance toe position.
/// Abstract State:    left support indicator.
/// Outputs:           continuous (minimal coordinates) state output
///                    floating-base state output
///
// TODO(Junda): class CAN be inherited

template <typename T>
class KneedCompassGait final : public systems::LeafSystem<T>{
 public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KneedCompassGait);

    /// Constructs the plant.
    explicit KneedCompassGait(RigidBodyTree<T> *tree);

    /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
    template <typename U>
    explicit KneedCompassGait(
            const KneedCompassGait<U>&) : KneedCompassGait<T>(nullptr) {}

    const systems::OutputPort<T>& get_minimal_state_output_port() const {
        return this->get_output_port(0);  // 8 output from state named vector
    }

    /// Returns reference to the output port that provides the state required by
    /// a RigidBodyTree loaded from CompassGait.urdf (as instantiated with
    /// FloatingBaseType::kRollPitchYaw).
    const systems::OutputPort<T>& get_floating_base_state_output_port() const {
        return this->get_output_port(1);  // 18 outputs of dofs
    }

    /// Returns the KneedCompassGaitContinuousState.
    static const KneedcompassgaitContinuousstate<T>& get_continuous_state(
            const systems::Context<T>& context) {
        return get_continuous_state(context.get_continuous_state());
    }

    /// Returns the mutable CompassGaitContinuousState.
    static KneedcompassgaitContinuousstate<T>& get_mutable_continuous_state(
            systems::Context<T>* context) {
        return get_mutable_continuous_state(
                &context->get_mutable_continuous_state());
    }

    // Set and Get two state: Discrete position of the stance toa;
    //  boolen indicator of "left support"
    static const T& get_toe_position(const systems::Context<T>& context) {
        return context.get_discrete_state(0).GetAtIndex(0);
    }

    static void set_toe_position(const T& value, systems::State<T>* state) {
        state->get_mutable_discrete_state().get_mutable_vector(0).SetAtIndex(0,
                value);
    }

    static bool left_leg_is_stance(const systems::Context<T> &context) {
        return context.template get_abstract_state<bool>(0);
    }

    static void set_left_leg_is_stance(bool value, systems::State<T>* state) {
        state->template get_mutable_abstract_state<bool>(0) = value;
    }

    /// Access the KneedCompassGaitParams.
    const KneedcompassgaitParams<T>& get_parameters(
            const systems::Context<T>& context) const {
        return this->template GetNumericParameter<
                KneedcompassgaitParams>(context, 0);
    }

// Sets the parameters to describe Kneed Compass Gait
// TODO(Junda): After all parameters decided, params_set API should be written
//    void SetKneedCWParams(systems::Parameters<T>* parameters) const;

    ///@{
    /// Manipulator equation of CompassGait: M(q)v̇ + bias(q,v) = 0.
    ///
    /// - M is the 4x4 mass matrix.
    /// - bias is a 4x1 vector that includes the Coriolis term and gravity term,
    ///   i.e. bias = C(q,v)*v - τ_g(q).

    RigidBodyTree<T>* rigidtree;
    Vector4<T> DynamicsBiasTerm(const systems::Context<T> &context) const;
    // MassMatrix had had parameter: const systems::Context<T> &context
//    Matrix4<T> MassMatrix(void) const;

private:
    static const KneedcompassgaitContinuousstate<T>& get_continuous_state(
            const systems::ContinuousState<T>& cstate) {
        return dynamic_cast<const KneedcompassgaitContinuousstate<T>&>(
                cstate.get_vector());
    }

    static KneedcompassgaitContinuousstate<T>& get_mutable_continuous_state(
            systems::ContinuousState<T>* cstate) {
        return dynamic_cast<KneedcompassgaitContinuousstate<T>&>(
                cstate->get_mutable_vector());
    }

    // Calculate the kinetic and potential energy (in the world frame attached to
    // the stance toe) AND now this is a simplified model
//    T DoCalcKineticEnergy(const systems::Context<T>& context) const final;
//    T DoCalcPotentialEnergy(const systems::Context<T>& context) const final;

// WitnessFunction to check when the foot hits the groud (with a sufficiently
    // large step length).
    T FootCollision(const systems::Context<T>& context) const;

//   Handles the impact dynamics, including resetting the stance and swing legs.
//    void CollisionDynamics(const systems::Context<T> &context,
//                           const systems::UnrestrictedUpdateEvent<T> &,
//                           systems::State<T> *state) const;

    // function pointers will be used when define the two ouput ports
    void MinimalStateOut(const systems::Context<T>& context,
                         KneedcompassgaitContinuousstate<T>* output) const;
    void FloatingBaseStateOut(const systems::Context<T>& context,
                              systems::BasicVector<T>* output) const;

    // Implements the simple double pendulum dynamics during stance.
    void DoCalcTimeDerivatives(
            const systems::Context<T>& context,
            systems::ContinuousState<T>* derivatives) const final;

//    void DoGetWitnessFunctions(const systems::Context<T>&,
//            std::vector<const systems::WitnessFunction<T>*>*
//                               witnesses) const final;

    // The system stores its witness function internally.
    std::unique_ptr<systems::WitnessFunction<T>> foot_collision_;


};
}   // namespace KneedCompassGait
}   // namespace examples
}   // namespace drake

#endif  // DRAKE_KNEEDCOMPASSGAIT_H
