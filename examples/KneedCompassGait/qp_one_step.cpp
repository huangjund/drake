//
// Created by Junda on 8/9/19.
//
#include <memory>
#include <string>

#include "drake/examples/KneedCompassGait/KCG_common.h"
#include "drake/examples/KneedCompassGait/qpController.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/scalar_conversion_traits.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/manipulation/util/sim_diagram_builder.h"



namespace drake {
namespace examples {
namespace qpControl {
    int doMain() {
        using systems::RigidBodyPlant;
        using manipulation::util::SimDiagramBuilder;
        using systems::DiagramBuilder;
        using systems::Simulator;
        using systems::ConstantVectorSource;
        using std::move;
        using std::cout;
        using std::endl;

        // get rigidbodytree pointer
        auto tree = std::make_unique<RigidBodyTree<double>>();
        tree = examples::kkk::getKCGTreed();

        // construct systems
        lcm::DrakeLcm lcm;
        SimDiagramBuilder<double> builder;
        DiagramBuilder<double> base_builder;

        // construct controller
        VectorX<double> q_des(NQ);
        q_des << -0.6, 0, 0, 0, -0.2, 0,
                0, 0.4, 0;

        // instantiate a rigidbodyplant from the rigidbodytree
        auto visulizer = base_builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
        auto& kcg = *base_builder.AddSystem<RigidBodyPlant<double>>(move(tree));
        examples::kkk::setDefaultContactParams(kcg);

        // connect kcg with visulizer
        base_builder.Connect(kcg.get_output_port(0),visulizer->get_input_port(0));
        // create zero source and connect it to kcg
//        VectorX<double> constant_vector(kcg.get_input_port(0).size());
//        constant_vector.setZero();
//        constant_vector[1] = 10;
//        auto constant0source = base_builder.AddSystem<
//                ConstantVectorSource<double>>(constant_vector);
//        constant0source->set_name("zero input");
//        base_builder.Connect(constant0source->get_output_port(),
//                kcg.get_input_port(0));
        auto qpcontroller = base_builder.AddSystem<qpController>(q_des);
        base_builder.Connect(qpcontroller->get_output_port(0),
                kcg.get_input_port(0));
        base_builder.Connect(kcg.get_output_port(0),
                qpcontroller->get_input_port(0));

        // build the diagram and set the simulator
        auto diagram = base_builder.Build();
        Simulator<double> simulator(*diagram);
        simulator.set_target_realtime_rate(0.1);
        simulator.set_publish_every_time_step(true);

        VectorX<double> x = examples::kkk::KCGFixedPointState();
        kcg.set_state_vector(&simulator.get_mutable_context(), x);

        simulator.Initialize();
        simulator.AdvanceTo(1);
        return 0;
    }
}
}
}

int main() {
    return drake::examples::qpControl::doMain();
}
