#ifndef DRAKE_KNEEDCOMPASSGAIT_LCM_H
#define DRAKE_KNEEDCOMPASSGAIT_LCM_H

#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to acrobot. The classes in this file are based on
/// iiwa_lcm.h

#include <memory>

#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_ContinuousState.h"
#include "drake/lcmt_KCG_u.hpp"
#include "drake/lcmt_KCG_x.hpp"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace KneedCompassGait {

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// kneed compass gait state channel with LCM type lcmt_KCG_x, and outputs the
/// kneed compass gait states as an KCGState.
///
/// @system{ KcgStateReceiver,
///   @input_port{lcmt_acrobot_x},
///   @output_port{acrobot_state} }
///
/// @ingroup kneed_compassgait_systems
    class KcgStateReceiver : public systems::LeafSystem<double> {
    public:
        KcgStateReceiver();

    private:
        void CopyStateOut(const systems::Context<double>& context,
                KneedcompassgaitContinuousstate<double>* output) const;
    };

/// Receives the output of an kcg controller, and outputs it as an LCM
/// message with type lcm_KCG_u. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
///
/// @system{ KCGCommandSender,
///   @input_port{torques},
///   @output_port{lcm_KCG_u} }
///
/// @ingroup KCG_systems
    class KcgCommandSender : public systems::LeafSystem<double> {
    public:
        KcgCommandSender();

    private:
        void OutputCommand(const systems::Context<double>& context,
                           lcmt_KCG_u* output) const;
    };

/// Receives the output of an LcmSubscriberSystem that subscribes to the
/// kneed compass gait input channel with LCM type lcmt_acrobot_u, and outputs the
/// kneed compass gait input as a BasicVector.
///
/// @system{ KcgCommandReceiver,
///   @input_port{lcmt_KCG_u},
///   @output_port{torques} }
///
/// @ingroup KCG_systems
    class KcgCommandReceiver : public systems::LeafSystem<double> {
    public:
        KcgCommandReceiver();

    private:
        void OutputCommandAsVector(const systems::Context<double>& context,
                                   systems::BasicVector<double>* output) const;
    };

/// Receives the output of an kcg_plant, and outputs it as an LCM
/// message with type lcm_KCG_x. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
///
/// @system{ KcgStateSender,
///   @input_port{Kcg_state},
///   @output_port{lcmt_Kcg_x} }
///
/// @ingroup KCg_systems
    class KcgStateSender : public systems::LeafSystem<double> {
    public:
        KcgStateSender();

    private:
        void OutputState(const systems::Context<double>& context,
                         lcmt_KCG_x* output) const;
    };

}  // namespace KneedCompassGait
}  // namespace examples
}  // namespace drake


#endif //DRAKE_KNEEDCOMPASSGAIT_LCM_H
