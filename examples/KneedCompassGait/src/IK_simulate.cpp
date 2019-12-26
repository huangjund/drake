#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"

#include "drake/examples/KneedCompassGait/include/phase_space_planner.h"
#include "drake/examples/KneedCompassGait/include/kneed_compass_gait.h"

using namespace std;

namespace drake {
namespace examples {
    
    int DoMain() {
        std::vector<double> step_lengths={0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
        std::vector<double> change_in_yaw={0, 0, 0, 0, 0, 0, 0, 0};
        std::vector<double> change_in_height={0, 0, 0, 0, 0, 0, 0, 0};
        std::vector<double> key_xdot={0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35};
        std::vector<double> key_height={0.68, 0.68, 0.68, 0.68, 0.68, 0.68, 0.68, 0.68};

        Eigen::Matrix<double, 5, 1> apex0;
        apex0 << 0, -0.115, 0.68, 0, 0.35;

        Eigen::Matrix<double, 5, 1> d0;
        d0 << 0, 0, 0.68, 0, 0.35;

        Eigen::Matrix<double, 3, 1> foot0;
        foot0 << 0, -0.2, 0;

        phase_space_planner::PhaseSpacePlanner psp;
        psp.Init(apex0, d0, foot0);
        for (size_t i = 0; i < step_lengths.size(); i++)
        {
            Action acn(step_lengths[i], change_in_yaw[i], change_in_height[i],  
            key_xdot[i], key_height[i]);
            psp.UpdatePrimitive(acn);
            psp.UpdateKeyframe();
            if(psp.step > 1){
            psp.CalOneStepTraj();
            }
        }
        
        string urdf_path = "drake/examples/KneedCompassGait/models/KneedCompassGait.urdf";

        auto tree = make_unique<RigidBodyTree<double>>();
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            FindResourceOrThrow(urdf_path), multibody::joints::kQuaternion, tree.get());

        // for (int i = 0; i < 2*tree->get_num_positions() - 1; i++)
        // {
        //     cout << tree->getStateName(i) << endl;
        // }
        
        systems::DiagramBuilder<double> builder;

        auto kcg = builder.AddSystem<kneed_compass_gait::KneedCompassGait>();
        kcg->InitSystem(urdf_path, psp.COM_list, psp.l_foot_list, psp.r_foot_list);

        lcm::DrakeLcm lcm;
        auto visulizer = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
        visulizer->set_name("visulizer");

        builder.Connect(kcg->get_output_port(0),visulizer->get_input_port(0));
        
        auto diagram = builder.Build();

        systems::Simulator<double> simulator(*diagram);
        

        simulator.Initialize();
        simulator.set_target_realtime_rate(0.1);
        simulator.AdvanceTo(1);

        return 0;

    }

}
}

int main() {
    return drake::examples::DoMain();
}
