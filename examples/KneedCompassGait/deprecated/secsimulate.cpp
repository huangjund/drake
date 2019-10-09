#include "drake/examples/KneedCompassGait/newKCG.h"
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include <memory>
#include <gflags/gflags.h>

namespace drake {
namespace examples {
namespace newKCG {
namespace {
    DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
    "Number of seconds to simulate.");
    DEFINE_string(urdf, "", "Name of urdf to load");
    DEFINE_bool(visualize_frames, true, "Visualize end effector frames");
    DEFINE_double(target_realtime_rate, 1.0,
    "Playback speed.  See documentation for "
    "Simulator::set_target_realtime_rate() for details.");

    using drake::manipulation::util::SimDiagramBuilder;
    using drake::systems::ConstantVectorSource;
    using drake::systems::Context;
    using drake::systems::Diagram;
    using drake::systems::DiagramBuilder;
    using drake::systems::FrameVisualizer;
    using drake::systems::RigidBodyPlant;
    using drake::systems::Simulator;

    int DoMain() {
        drake::lcm::DrakeLcm lcm;
        SimDiagramBuilder<double> builder;

        RigidBodyPlant<double>* plant = nullptr;
        {
            auto tree = getKCGTreed();
            auto plantDt =
                    std::make_unique<RigidBodyPlant<double>>(std::move(tree), 0.0005);
            plant = builder.AddPlant(std::move(plantDt));
        }

        // Create LCM publisher for visualization
        builder.AddVisualizer(&lcm);
        builder.get_visualizer()->set_publish_period(1e-2);

        // contact parameters
        setDefaultContactParams(*plant);

        drake::systems::DiagramBuilder<double> *base_builder =
                builder.get_mutable_builder();

        VectorX<double> constant_vector(plant->get_input_port(0).size());
        constant_vector.setZero();
        constant_vector[0] = 0.1;
        auto constant_zero_source =
                base_builder->AddSystem<ConstantVectorSource<double>>(constant_vector);
        constant_zero_source->set_name("zero input");

        // Connects the blank input command
        base_builder->Connect(constant_zero_source->get_output_port(),
                              plant->get_input_port(0));

        auto sys = builder.Build();

        Simulator<double> simulator(*sys);

        lcm.HandleSubscriptions(10);
        simulator.set_publish_every_time_step(true);
        simulator.set_target_realtime_rate(1);

        // simulator.get_mutable_context().get_mutable_continuous_state().
        //       SetFromVector(CassieFixedPointState());
        plant->set_state_vector(&simulator.get_mutable_context(),
                                KCGFixedPointState());
        simulator.Initialize();

//        auto context = plant->CreateDefaultContext();
//        std::cout << plant->get_num_positions() << std::endl;
//        std::cout << plant->get_output_port(0).Eval(*context) << std::endl;


        simulator.AdvanceTo(10.0);
        return 0;
    }
}
}  // namespace newKCG
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]){
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::examples::newKCG::DoMain();
}
