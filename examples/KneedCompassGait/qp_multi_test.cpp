//
// Created by Junda Huang on 9/2/19.
//
#include <memory>
#include <string>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include "drake/examples/KneedCompassGait/KCG_common.h"
#include "drake/examples/KneedCompassGait/qpControllertest.h"
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
#include "drake/systems/primitives/signal_logger.h"
#include "drake/examples/KneedCompassGait/system.h"

#define TIME_RATE 0.01
#define SIMULATION_TIME 1.5

namespace drake {
    namespace examples {
        namespace qpControl {
            int writeCSV(Eigen::MatrixXd data, std::string& kFile) {
                std::ofstream outFile;
                outFile.open(kFile, std::ios::out);
                outFile << data << std::endl;
                outFile.close();

                return 1;
            }
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

                // construct controller desired intput
//        VectorX<double> Xcom(COM);
//        Xcom << 0, 0, 0.65, -1, 0, 0;

                // instantiate a rigidbodyplant from the rigidbodytree
                auto visulizer = base_builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm, true);
                auto& kcg = *base_builder.AddSystem<RigidBodyPlant<double>>(move(tree));
                examples::kkk::setDefaultContactParams(kcg);

                // construct the controller
                auto qpcontroller = base_builder.AddSystem<qpController>();

                // construct the motion generator
                // the real output should add a minus
                Eigen::Matrix<double, 6, 1> U;
                U << 1.25, 0, 0, 0, 0, 0;
                auto motion = base_builder.AddSystem<linear_system::LinearSystem>(U);
                Eigen::Matrix<double, 6, 1> X0;
                X0 << 1, 0, 0.65, 1, 0, 0;
                std::vector<double> step_lengths = {0.5, 0.5, 0.5, 0.5};
                std::vector<double> change_in_yaw = {0, 0, 0, 0};
                motion->set_name("motion generator");

//        // create zero source and connect it to kcg
//        VectorX<double> constant_vector(qpcontroller->get_input_port(1).size());
//        constant_vector.setZero();
//        constant_vector = Xcom;
//        auto constant0source = base_builder.AddSystem<
//                ConstantVectorSource<double>>(constant_vector);
//        constant0source->set_name("trajectory_input");

                auto logger1 = systems::LogOutput(kcg.get_output_port(0), &base_builder);
                auto logger2 = systems::LogOutput(qpcontroller->get_output_port(0), &base_builder);

                // connect
                base_builder.Connect(kcg.get_output_port(0),visulizer->get_input_port(0));
                base_builder.Connect(qpcontroller->get_output_port(0),
                                     kcg.get_input_port(0));
                base_builder.Connect(kcg.get_output_port(0),
                                     qpcontroller->get_input_port(0));
                base_builder.Connect(motion->get_output_port(0),
                                     qpcontroller->get_input_port(1));
                base_builder.Connect(motion->get_output_port(1),
                                     qpcontroller->get_input_port(2));

                // build the diagram and set the simulator
                auto diagram = base_builder.Build();
                Simulator<double> simulator(*diagram);
                auto& motion_mutable_context = diagram->GetMutableSubsystemContext(
                        *motion, &simulator.get_mutable_context());
                auto& state = motion_mutable_context.get_mutable_continuous_state();
                auto& kcg_mutable_context = diagram->GetMutableSubsystemContext(
                        kcg, &simulator.get_mutable_context());

                motion->SetInitState(X0, state, step_lengths, change_in_yaw);
                motion->SetInput(U);
                simulator.set_target_realtime_rate(TIME_RATE);
                simulator.set_publish_every_time_step(true);

                VectorX<double> x = examples::kkk::KCGFixedPointState();
                kcg.set_state_vector(&kcg_mutable_context, x);
                simulator.Initialize();

                simulator.AdvanceTo(SIMULATION_TIME);

                Eigen::MatrixXd data1 = logger1->data();
                Eigen::MatrixXd sampleTime1 = logger1->sample_times();
                std::string kFile;
                kFile = "data1.csv";
                writeCSV(data1, kFile);
                kFile = "time1.csv";
                writeCSV(sampleTime1, kFile);

                Eigen::MatrixXd data2 = logger2->data();
                Eigen::MatrixXd sampleTime2 = logger2->sample_times();
                kFile = "data2.csv";
                writeCSV(data2, kFile);
                kFile = "time2.csv";
                writeCSV(sampleTime2, kFile);

                while(true)
                    visulizer->ReplayCachedSimulation();
                return 0;
            }
        }
    }
}

int main() {
    return drake::examples::qpControl::doMain();
}

