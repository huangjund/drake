#include <memory>
#include <vector>

#include <gflags/gflags.h>

#include "drake/examples/KneedCompassGait/KCG_common.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rotation_matrix.h"
#include "drake/common/find_resource.h"
#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_ContinuousState.h"
#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_Params.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/scalar_conversion_traits.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/witness_function.h"
#include "drake/systems/primitives/constant_vector_source.h"

#define NQ 11
#define NV 11
#define NU 5
#define STATE 22

namespace drake {
namespace examples {
namespace kkk {
    class kkkcg : public systems::LeafSystem<double> {
    public:
        kkkcg(){
            // Four Continuous State
            this->DeclareContinuousState(
                    KneedCompassGait::KneedcompassgaitContinuousstate<double>(), NQ, NV, 0);

            // Discrete state for left toe distance along the plane.
            this->DeclareDiscreteState(1);

            // Abstract state for indicating the left leg is the stance leg. This
            // is only used for the visualization output.
            bool left_stance = true;
            this->DeclareAbstractState(AbstractValue::Make(left_stance));

            // The floating-base state of the system(useful for visual)
            this->DeclareVectorOutputPort(systems::BasicVector<double>(STATE),
                                          &kkkcg::FloatingBaseStateOut);

            // Natural property
            this->DeclareNumericParameter(
                    KneedCompassGait::KneedcompassgaitParams<double>());

//            foot_collision = this->MakeWitnessFunction(
//                    "foot collsion",
//                    systems::WitnessFunctionDirection::kPositiveThenNonPositive,
//                    &kkkcg::FootCollision,
//                    &kkkcg::CollisionDynamics
//                    );

        }

        const KneedCompassGait::KneedcompassgaitParams<double>& get_parameters(
                const systems::Context<double>& context) const {
            return this->template GetNumericParameter<
                    KneedCompassGait::KneedcompassgaitParams>(context, 0);
        }

        static const KneedCompassGait::KneedcompassgaitContinuousstate<double>&
                get_continuous_state(const systems::Context<double>& context) {
            return get_continuous_state(context.get_continuous_state());
        }

        static KneedCompassGait::KneedcompassgaitContinuousstate<double>&
                get_mutable_continuous_state(systems::Context<double>* context) {
            return get_mutable_continuous_state(
                    &context->get_mutable_continuous_state());
        }

        static const double& get_toe_position(
                const systems::Context<double>& context) {
            return context.get_discrete_state(0).GetAtIndex(0);
        }

        static bool left_leg_is_stance(const systems::Context<double> &context) {
            return context.template get_abstract_state<bool>(0);
        }

        void FloatingBaseStateOut(const systems::Context<double>& context,
                systems::BasicVector<double>* output) const {
            const KneedCompassGait::KneedcompassgaitContinuousstate<double>&
                    cs = get_continuous_state(context);
//            const KneedCompassGait::KneedcompassgaitParams<double>&
//                    p = get_parameters(context);
            const bool left_stance = left_leg_is_stance(context);

//// TODO(Junda): need to be modified
            output->SetAtIndex(0, cs.x());
            output->SetAtIndex(1, cs.y());
            output->SetAtIndex(2, cs.z());
            output->SetAtIndex(3, cs.roll());
            output->SetAtIndex(4, cs.pitch());
            output->SetAtIndex(5, cs.yaw());

            output->SetAtIndex(11, cs.xdot());
            output->SetAtIndex(12, cs.ydot());
            output->SetAtIndex(13, cs.zdot());
            output->SetAtIndex(14, cs.wr());
            output->SetAtIndex(15, cs.wp());
            output->SetAtIndex(16, cs.wy());

            const double left = left_stance ?
                           cs.angle_stance_knee() : cs.angle_swing_knee();
            const double right = left_stance ?
                        cs.angle_swing_knee() : cs.angle_stance_knee();
            const double hiproll = left_stance ? cs.angle_hiprow() : -cs.angle_hiprow();
            const double hippin = left_stance ? cs.angle_hippin() : -cs.angle_hippin();
            const double hipyaw = left_stance ? cs.angle_hipyaw() : -cs.angle_hipyaw();

            output->SetAtIndex(6, left);
            output->SetAtIndex(7, hiproll);
            output->SetAtIndex(8, hippin);
            output->SetAtIndex(9, hipyaw);
            output->SetAtIndex(10, right);

            const double leftdot = left_stance ?
                    cs.angledot_stance_knee() : cs.angledot_swing_knee();
            const double rightdot = left_stance ?
                    cs.angledot_swing_knee() : cs.angledot_stance_knee();
            const double hipdotroll = left_stance ?
                    cs.angledot_hiprow() : -cs.angledot_hiprow();
            const double hipdotpin = left_stance ?
                    cs.angledot_hippin() : -cs.angledot_hippin();
            const double hipdotyaw = left_stance ?
                    cs.angledot_hipyaw() : -cs.angledot_hipyaw();

            output->SetAtIndex(17, leftdot);
            output->SetAtIndex(18, hipdotroll);
            output->SetAtIndex(19, hipdotpin);
            output->SetAtIndex(20, hipdotyaw);
            output->SetAtIndex(21, rightdot);
        }

        template <typename Scalar>
        std::unique_ptr<RigidBodyTree<Scalar>> getkkkcgTree() const {
            auto tree = std::make_unique<RigidBodyTree<Scalar>>();
            std::string urdf_path =
                    "drake/examples/KneedCompassGait/KneedCompassGait.urdf";

            drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                    FindResourceOrThrow(urdf_path), multibody::joints::kRollPitchYaw,
                    tree.get()
                    );

            drake::multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);

            return tree;
        }

    private:
        void DoCalcTimeDerivatives(const systems::Context<double>& context,
                systems::ContinuousState<double>* derivatives) const override{
            auto tree = std::make_unique<RigidBodyTree<double>>();
            tree = getkkkcgTree<double>();

            Eigen::VectorXd nq(NQ), nv(NV) ,temp(STATE);
            temp = context.get_continuous_state().CopyToVector();
            nq << temp.segment(0, NQ-1);
            nv << temp.segment(NQ, STATE-1);
            auto kinsol = tree->doKinematics(nq, nv);

            auto M = tree->massMatrix(kinsol);
            auto C_bias = tree->dynamicsBiasTerm(kinsol, {});


            VectorX<double> xdot(22);
            xdot << nv, -M.inverse()*C_bias;
            std::cout << M << std::endl;
            derivatives->SetFromVector(xdot);
        };

        double DoCalcKineticEnergy(
                const systems::Context<double>& context) const override {
            std::cout << "DoCalcKineticEnergy 1 excuted" << std::endl;
            VectorX<double> x = context.get_continuous_state().CopyToVector();

            auto tree = std::make_unique<RigidBodyTree<double>>();
            tree = getkkkcgTree<double>();

            Eigen::VectorXd nq(NQ), nv(NV) ,temp(STATE);
            temp = context.get_continuous_state().CopyToVector();
            nq << temp.segment(0, NQ-1);
            nv << temp.segment(NQ, STATE-1);
            auto kinsol = tree->doKinematics(nq, nv);

            VectorX<double> T(1);
            T << (nq.transpose()*(tree->massMatrix(kinsol))*nq)/2.;
            std::cout << "DoCalcKineticEnergy 2 excuted" << std::endl;
            return T[1];
        }

        static const KneedCompassGait::KneedcompassgaitContinuousstate<double>&
        get_continuous_state(const systems::ContinuousState<double>& cstate) {
            return dynamic_cast<
                    const KneedCompassGait::KneedcompassgaitContinuousstate<double>&>(
                    cstate.get_vector());
        }

        static KneedCompassGait::KneedcompassgaitContinuousstate<double>&
                get_mutable_continuous_state(systems::ContinuousState<double>* cstate) {
            return dynamic_cast<KneedCompassGait::KneedcompassgaitContinuousstate<double>&>(
                    cstate->get_mutable_vector());
        }

    };

    int main() {
        using manipulation::util::SimDiagramBuilder;
        using systems::DiagramBuilder;
        using systems::RigidBodyPlant;
        using systems::Simulator;
        using systems::ConstantVectorSource;

        drake::lcm::DrakeLcm lcm;
        SimDiagramBuilder<double> builder;
        RigidBodyPlant<double>* plant = nullptr;

        // Create Plant
        auto tree = std::make_unique<RigidBodyTree<double>>();
        tree = getKCGTreed();
        plant = builder.AddPlant(std::move(tree));

        // Create LCM publisher for visualization
        builder.AddVisualizer(&lcm);
        builder.get_visualizer()->set_publish_period(1e-2);

        // contact parameters
        setDefaultContactParams(*plant);

        drake::systems::DiagramBuilder<double> *base_builder =
                builder.get_mutable_builder();

        VectorX<double> constant_vector(plant->get_input_port(0).size());
        constant_vector.setZero();
       // constant_vector[0] = 10;
        auto constant_zero_source =
                base_builder->AddSystem<ConstantVectorSource<double>>(constant_vector);
        constant_zero_source->set_name("zero input");

        // Connects the blank input command
        base_builder->Connect(constant_zero_source->get_output_port(),
                              plant->get_input_port(0));

        auto sys = builder.Build();

        Simulator<double> simulator(*sys);

      //  lcm.HandleSubscriptions(10);
        simulator.set_publish_every_time_step(true);
        simulator.set_target_realtime_rate(0.1);
        simulator.get_mutable_context().SetAccuracy(1e-4);

        plant->set_state_vector(&simulator.get_mutable_context(),
                                KCGFixedPointState());
        simulator.Initialize();

        simulator.AdvanceTo(1);
        return 0;
    }
} // kkk
} // examples
} // drake

int main() {
    return drake::examples::kkk::main();
}
