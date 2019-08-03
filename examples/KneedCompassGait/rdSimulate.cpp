#include <memory>
#include <vector>

#include <gflags/gflags.h>

#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_ContinuousState.h"
#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_Params.h"
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rotation_matrix.h"
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


namespace drake {
namespace examples {
namespace kkk {
    class kkkcg : public systems::LeafSystem<double> {
    public:
        kkkcg(){
            // Four Continuous State
            this->DeclareContinuousState(
                    KneedCompassGait::KneedcompassgaitContinuousstate<double>(), 9, 9, 0);

            // Discrete state for left toe distance along the plane.
            this->DeclareDiscreteState(1);

            // Abstract state for indicating the left leg is the stance leg. This
            // is only used for the visualization output.
            bool left_stance = true;
            this->DeclareAbstractState(AbstractValue::Make(left_stance));

            // The floating-base state of the system(useful for visual)
            this->DeclareVectorOutputPort(systems::BasicVector<double>(18),
                                          &kkkcg::FloatingBaseStateOut);

            // Natural property
            this->DeclareNumericParameter(
                    KneedCompassGait::KneedcompassgaitParams<double>());

            H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 1, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 1, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 1,
                 0, 0, 0, 0, 1, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 1, 0;

            Ht = H.transpose();
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

        static const double& get_toe_position(
                const systems::Context<double>& context) {
            return context.get_discrete_state(0).GetAtIndex(0);
        }

        static bool left_leg_is_stance(const systems::Context<double> &context) {
            return context.template get_abstract_state<bool>(0);
        }

        void FloatingBaseStateOut(
                const systems::Context<double>& context,
                systems::BasicVector<double>* output
                ) const {
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

            output->SetAtIndex(9, cs.xdot());
            output->SetAtIndex(10, cs.ydot());
            output->SetAtIndex(11, cs.zdot());
            output->SetAtIndex(12, cs.wr());
            output->SetAtIndex(13, cs.wp());
            output->SetAtIndex(14, cs.wy());

            const double left = left_stance ?
                           cs.angle_stance_knee() : cs.angle_swing_knee();
            const double right = left_stance ?
                        cs.angle_swing_knee() : cs.angle_stance_knee();
            const double hip = left_stance ? cs.angle_hip() : -cs.angle_hip();

            output->SetAtIndex(6, left);
            output->SetAtIndex(7, hip);
            output->SetAtIndex(8, right);

            const double leftdot = left_stance ?
                    cs.angledot_stance_knee() : cs.angledot_swing_knee();
            const double rightdot = left_stance ?
                    cs.angledot_swing_knee() : cs.angledot_stance_knee();
            const double hipdot = left_stance ?
                    cs.angledot_hip() : -cs.angledot_hip();

            output->SetAtIndex(15, leftdot);
            output->SetAtIndex(16, hipdot);
            output->SetAtIndex(17, rightdot);
        }

        template <typename Scalar>
        std::unique_ptr<RigidBodyTree<Scalar>>
        getkkkcgTree() const {
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
        Eigen::Matrix<double, 6, 9> H;
        Eigen::Matrix<double, 9, 6> Ht;
        void DoCalcTimeDerivatives(
                const systems::Context<double>& context,
                systems::ContinuousState<double>* derivatives) const override{
            auto tree = std::make_unique<RigidBodyTree<double>>();
            tree = getkkkcgTree<double>();

            Eigen::VectorXd nq(9), nv(9) ,temp;
            nq = context.get_continuous_state().CopyToVector();
            auto kinsol = tree->doKinematics(nq, nv);


            VectorX<double> xdot(18);
            derivatives->SetFromVector(xdot);
        };

        static const KneedCompassGait::KneedcompassgaitContinuousstate<double>&
                get_continuous_state(const systems::ContinuousState<double>& cstate) {
            return dynamic_cast<
            const KneedCompassGait::KneedcompassgaitContinuousstate<double>&>(
                    cstate.get_vector());
        }
    };

    int main() {
        kkkcg basic_system;
        systems::Simulator<double> simulator(basic_system);

        systems::ContinuousState<double>& state =
                simulator.get_mutable_context().get_mutable_continuous_state();

        state[0] = 0.9;

        simulator.AdvanceTo(1);

        return 0;
    }
} // kkk
} // examples
} // drake

int main() {
    return drake::examples::kkk::main();
}
