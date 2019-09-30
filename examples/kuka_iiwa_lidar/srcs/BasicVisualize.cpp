//
// Created by Junda Huang on 9/29/19.
//

#include "drake/examples/kuka_iiwa_lidar/includes/KUKA_Common.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace examples {
namespace KUKA {
    int DoMain() {
        using manipulation::util::SimDiagramBuilder;
        using systems::DiagramBuilder;
        using systems::Simulator;
        using geometry::SceneGraph;
        using multibody::MultibodyPlant;
        using systems::RigidBodyPlant;
        using std::cout;
        using std::endl;

        drake::lcm::DrakeLcm lcm;
        SimDiagramBuilder<double> builder;
//        RigidBodyPlant<double>* plant = nullptr;

        // Create Plant
        auto tree = std::make_unique<RigidBodyTree<double>>();
        tree = getKUKATreed();
        builder.AddPlant(std::move(tree));

        // Create LCM publisher for visualization
        builder.AddVisualizer(&lcm);
        builder.get_visualizer()->set_publish_period(1e-2);

        auto sys = builder.Build();

        Simulator<double> simulator(*sys);

        simulator.set_publish_every_time_step(true);
        simulator.set_target_realtime_rate(0.1);
        simulator.get_mutable_context().SetAccuracy(1e-4);

//        drake::systems::DiagramBuilder<double> *base_builder =
//                builder.get_mutable_builder();

//        plant->set_state_vector(&simulator.get_mutable_context(),
//                                KCGFixedPointState());
        simulator.Initialize();

        simulator.AdvanceTo(0.25);
        return 1;
    }
}
}
}

int main() {
    return drake::examples::KUKA::DoMain();
}
