#include "drake/examples/KneedCompassGait/KneedCompassGait_lcm.h"

#include "drake/lcmt_KCG_u.hpp"
#include "drake/lcmt_KCG_x.hpp"

namespace drake {
namespace examples {
namespace KneedCompassGait {
    using systems::Context;
    using systems::SystemOutput;

    static const int kNumJoints = 3;

/*--------------------------------------------------------------------------*/
// methods implementation for KcgStateReceiver.

    KcgStateReceiver::KcgStateReceiver(){
        this->DeclareAbstractInputPort("lcmt_KCG_x",
                                       Value<lcmt_KCG_x>{});
        this->DeclareVectorOutputPort("kneed_compass_gait_state",
                                      &KcgStateReceiver::CopyStateOut);
    }

    void KcgStateReceiver::CopyStateOut(const Context<double>& context,
            KneedcompassgaitContinuousstate<double>* output) const {
        const AbstractValue* input = this->EvalAbstractInput(context, 0);
        DRAKE_ASSERT(input != nullptr);
        const auto& state = input->get_value<lcmt_KCG_x>();
        auto output_vec = output->get_mutable_value();

        output_vec(0) = state.stance_fai;
        output_vec(1) = state.swing_fai;
        output_vec(2) = state.hip_fai;
        output_vec(3) = state.thigh_fai;
        output_vec(4) = state.stancedot_fai;
        output_vec(5) = state.swingdot_fai;
        output_vec(6) = state.hipdot_fai;
        output_vec(7) = state.thighdot_fai;
    }

/*--------------------------------------------------------------------------*/
// methods implementation for KcgCommandSender.

    KcgCommandSender::KcgCommandSender() {
        this->DeclareInputPort("torques", systems::kVectorValued, kNumJoints);
        this->DeclareAbstractOutputPort("lcmt_KCG_u",
                                        &KcgCommandSender::OutputCommand);
    }

    void KcgCommandSender::OutputCommand(const Context<double>& context,
                                             lcmt_KCG_u* status) const {
        const systems::BasicVector<double>* command =
                this->EvalVectorInput(context, 0);

        status->hip_tau = command->GetAtIndex(0);
        status->leftknee_tau = command->GetAtIndex(1);
        status->rightknee_tau = command->GetAtIndex(2);
    }

/*--------------------------------------------------------------------------*/
// methods implementation for KcgCommandReceiver

    KcgCommandReceiver::KcgCommandReceiver() {
        this->DeclareAbstractInputPort("lcmt_KCG_u",
                                       Value<lcmt_KCG_u>());
        this->DeclareVectorOutputPort("torques",
                systems::BasicVector<double>(kNumJoints),
                        &KcgCommandReceiver::OutputCommandAsVector);
    }

    void KcgCommandReceiver::OutputCommandAsVector(
            const Context<double>& context,
            systems::BasicVector<double>* output) const {
        const AbstractValue* input = this->EvalAbstractInput(context, 0);

        DRAKE_ASSERT(input != nullptr);
        const auto& command = input->get_value<lcmt_KCG_u>();
        output->SetAtIndex(0, command.hip_tau);
        output->SetAtIndex(1, command.leftknee_tau);
        output->SetAtIndex(2, command.rightknee_tau);
    }

/*--------------------------------------------------------------------------*/
// methods implementation for KcgStateSender

    KcgStateSender::KcgStateSender() {
        this->DeclareInputPort("kneed_compass_gait_state",
                systems::kVectorValued, 8);
        this->DeclareAbstractOutputPort("lcmt_KCG_x",
                                        &KcgStateSender::OutputState);
    }

    void KcgStateSender::OutputState(const Context<double>& context,
                                         lcmt_KCG_x* status) const {
        const systems::BasicVector<double>* state = this->EvalVectorInput(context, 0);
        status->stance_fai = state->GetAtIndex(0);
        status->swing_fai = state->GetAtIndex(1);
        status->hip_fai = state->GetAtIndex(2);
        status->thigh_fai = state->GetAtIndex(3);
        status->stancedot_fai = state->GetAtIndex(4);
        status->swingdot_fai = state->GetAtIndex(5);
        status->hipdot_fai = state->GetAtIndex(6);
        status->thighdot_fai = state->GetAtIndex(7);
    }

}  // namespace KneedCompassGait
}  // namespace examples
}  // namespace drake
