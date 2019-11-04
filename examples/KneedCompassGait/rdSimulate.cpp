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
#include "drake/systems/primitives/signal_logger.h"

#define NQ 12
#define NV 12
#define NU 6
#define STATE 24

namespace drake {
namespace examples {
namespace kkk {
    int writeCSV(Eigen::MatrixXd data, std::string& kFile) {
        std::ofstream outFile;
        outFile.open(kFile, std::ios::out);
        outFile << data << std::endl;
        outFile.close();

        return 1;
    }
    void printCOM(const RigidBodyTree<double>& r) {
        auto ret = KCGFixedPointState();

        VectorX<double> q, qd;
        q = ret.segment(0,NQ);
        qd = ret.segment(NQ,NV);
        auto kinsol = r.doKinematics(q, qd);
        std::cout << r.centerOfMass(kinsol) << std::endl;
    }
    int main() {
        using manipulation::util::SimDiagramBuilder;
        using systems::DiagramBuilder;
        using systems::RigidBodyPlant;
        using systems::Simulator;
        using systems::ConstantVectorSource;
        using std::cout;
        using std::endl;

        drake::lcm::DrakeLcm lcm;
        SimDiagramBuilder<double> builder;
        RigidBodyPlant<double>* plant = nullptr;

        // Create Plant
        auto tree = std::make_unique<RigidBodyTree<double>>();
        tree = getKCGTreed();
        printCOM(*tree);
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
        constant_vector[0] = 0;
        constant_vector[1] = 0;
        constant_vector[2] = 0;
        constant_vector[3] = 0;
        constant_vector[4] = 0;
        constant_vector[5] = 0;
        auto constant_zero_source =
                base_builder->AddSystem<ConstantVectorSource<double>>(constant_vector);
        constant_zero_source->set_name("zero input");

        // Connects the blank input command
        base_builder->Connect(constant_zero_source->get_output_port(),
                              plant->get_input_port(0));

        auto logger1 = systems::LogOutput(plant->get_output_port(0), base_builder);
   //     auto logger2 = systems::LogOutput(plant->contact_results_output_port(), base_builder);

        auto sys = builder.Build();

        Simulator<double> simulator(*sys);

      //  lcm.HandleSubscriptions(10);
        simulator.set_publish_every_time_step(true);
        simulator.set_target_realtime_rate(0.1);
        simulator.get_mutable_context().SetAccuracy(1e-4);

        plant->set_state_vector(&simulator.get_mutable_context(),
                                KCGFixedPointState());

        simulator.Initialize();

        simulator.AdvanceTo(0.25);

        Eigen::MatrixXd data1 = logger1->data();
        Eigen::MatrixXd sampleTime1 = logger1->sample_times();
        std::string kFile;
        kFile = "data1.csv";
        writeCSV(data1, kFile);
        kFile = "time1.csv";
        writeCSV(sampleTime1, kFile);
//        Eigen::MatrixXd data2 = logger2->data();
//        Eigen::MatrixXd sampleTime2 = logger2->sample_times();
//        cout << "data2\n" << data2 << endl;
//        cout << "sample2\n" << sampleTime2 << endl;
//        kFile = "data2.csv";
//        writeCSV(data2, kFile);
//        kFile = "time2.csv";
//        writeCSV(sampleTime2, kFile);
        return 0;
    }
} // kkk
} // examples
} // drake

int main() {
    return drake::examples::kkk::main();
}
