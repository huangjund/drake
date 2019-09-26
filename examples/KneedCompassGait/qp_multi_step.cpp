//
// Created by Junda on 8/20/19.
//
#include <memory>
#include <string>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include "drake/examples/KneedCompassGait/KCG_common.h"
#include "drake/examples/KneedCompassGait/qpController2.h"
#include "drake/examples/KneedCompassGait/simulator.h"
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
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

#define TIME_RATE 0.5
#define SIMULATION_TIME 6
#define BOUND 0.14

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


        auto logger1 = systems::LogOutput(kcg.get_output_port(0), &base_builder);
        auto logger2 = systems::LogOutput(qpcontroller->get_output_port(0), &base_builder);
        auto logger3 = systems::LogOutput(qpcontroller->get_output_port(1), &base_builder);

        // connect
        base_builder.Connect(kcg.get_output_port(0),visulizer->get_input_port(0));
        base_builder.Connect(qpcontroller->get_output_port(0),
                             kcg.get_input_port(0));
        base_builder.Connect(kcg.get_output_port(0),
                             qpcontroller->get_input_port(0));

        // build the diagram and set the simulator
        auto diagram = base_builder.Build();
        Simulator<double> simulator(*diagram);

        auto& kcg_mutable_context = diagram->GetMutableSubsystemContext(
                kcg, &simulator.get_mutable_context());

        simulator.set_target_realtime_rate(TIME_RATE);
        simulator.set_publish_every_time_step(true);

        VectorX<double> x = examples::kkk::KCGFixedPointState();
        kcg.set_state_vector(&kcg_mutable_context, x);
        stx::optional<double> accuracy = 1e-14;
        simulator.get_mutable_context().SetAccuracy(accuracy);
        double c_time=0;// ,d_time=0;

        auto& last_sim_context = simulator.get_context();
        auto& last_kcg_context = diagram->GetSubsystemContext(kcg, last_sim_context);
        auto& last_qp_context = diagram->GetSubsystemContext(*qpcontroller, last_sim_context);
        auto last_kcg_c_states = last_kcg_context.get_continuous_state().CopyToVector();
        auto last_qp_d_states = last_qp_context.get_discrete_state().get_vector().CopyToVector();
        auto& last_qp_a_states = last_qp_context.get_abstract_state();

        // last abstract states of qp controller
        std::vector<std::unique_ptr<AbstractValue>> abstract_vector;
        abstract_vector.push_back(last_qp_a_states.get_value(0).Clone()); // period_state_
        abstract_vector.push_back(last_qp_a_states.get_value(1).Clone()); // sim_tim_
        abstract_vector.push_back(last_qp_a_states.get_value(2).Clone()); // t
        abstract_vector.push_back(last_qp_a_states.get_value(3).Clone()); // t_pre_
        abstract_vector.push_back(last_qp_a_states.get_value(4).Clone()); // m_
        abstract_vector.push_back(last_qp_a_states.get_value(5).Clone()); // count_num_
        abstract_vector.push_back(last_qp_a_states.get_value(6).Clone()); // count_num_change_
        abstract_vector.push_back(last_qp_a_states.get_value(7).Clone()); // step_num_
        abstract_vector.push_back(last_qp_a_states.get_value(8).Clone()); // replanning
        abstract_vector.push_back(last_qp_a_states.get_value(9).Clone()); // isNew_
        abstract_vector.push_back(last_qp_a_states.get_value(10).Clone()); // stance_now_
        abstract_vector.push_back(last_qp_a_states.get_value(11).Clone()); // location_bias_
        abstract_vector.push_back(last_qp_a_states.get_value(12).Clone()); // modified_
        abstract_vector.push_back(last_qp_a_states.get_value(13).Clone()); // incline_
        abstract_vector.push_back(last_qp_a_states.get_value(14).Clone()); // refreshed_
        abstract_vector.push_back(last_qp_a_states.get_value(15).Clone()); // incline_
        abstract_vector.push_back(last_qp_a_states.get_value(16).Clone()); // refreshed_

        systems::AbstractValues last_AStates(std::move(abstract_vector));

        double last_c_time = c_time;
        int LastTimeCase = 1;
        double bias_lower_bound = -BOUND;
        double bias_higher_bound = BOUND;
        double location_bias = 0;
        bool case1_ticket = false;

        simulator.Initialize();
        simulator.AdvanceTo(SIMULATION_TIME);

        int i = 1000;
        while (i--) {
            // current state obtain
            auto& sim_context = simulator.get_context();
            auto& kcg_context = diagram->GetSubsystemContext(kcg, sim_context);
            auto& qp_context = diagram->GetSubsystemContext(*qpcontroller, sim_context);
            auto kcg_c_states = kcg_context.get_continuous_state().CopyToVector();
            auto qp_d_states = qp_context.get_discrete_state().get_vector().CopyToVector();
            auto& qp_a_states = qp_context.get_abstract_state();
            c_time = sim_context.get_time(); // get the time
            int incline = qp_a_states.get_value(13).get_value<int>();
            int period_state = qp_a_states.get_value(0).get_value<int>();
            bool refresh = false;

            switch (period_state) {
                case 4: { // recorde mode
                    location_bias = 0;
                    refresh = true;
                    std::cout << "case4" << std::endl;
                    // state pull
                    auto& sim_mutable_context = simulator.get_mutable_context();
                    auto& KCG_mutable_context = diagram->GetMutableSubsystemContext(kcg, &sim_mutable_context);
                    auto& qp_mutable_context = diagram->GetMutableSubsystemContext(*qpcontroller, &sim_mutable_context);
                    auto& qp_mutable_AStates = qp_mutable_context.get_mutable_abstract_state();
                    kcg.set_state_vector(&KCG_mutable_context, kcg_c_states);
                    qp_mutable_AStates.SetFrom(qp_a_states);

                    qp_mutable_AStates.get_mutable_value(11).set_value<double>(location_bias);
                    qp_mutable_AStates.get_mutable_value(12).set_value<bool>(true); // set as modified
                    qp_mutable_AStates.get_mutable_value(0).set_value<int>(2); // reset the period_state
                    qp_mutable_AStates.get_mutable_value(14).set_value<bool>(refresh);
                    sim_mutable_context.SetTime(c_time);

                    // current states becomes last states
                    last_kcg_c_states = kcg_c_states; // current kcg continue state  assign to last continue state
                    last_qp_d_states = qp_d_states; // current qp discrete state assign to last discrete state
                    last_AStates.SetFrom(qp_a_states);// 0 extract that reset the period_state; 12extract that set as modified

                    last_c_time = c_time;
                    LastTimeCase = 1;
                    bias_higher_bound = BOUND;
                    bias_lower_bound = -BOUND;
                    break;
                }
                case 1: { // preceeding one piece mode
                    refresh = true;
                    case1_ticket = true;
                    std::cout << "case1" << std::endl;
                    auto& sim_mutable_context = simulator.get_mutable_context();
                    auto& KCG_mutable_context = diagram->GetMutableSubsystemContext(kcg, &sim_mutable_context);
                    auto& qp_mutable_context = diagram->GetMutableSubsystemContext(*qpcontroller, &sim_mutable_context);
                    auto& qp_mutable_AStates = qp_mutable_context.get_mutable_abstract_state();

                    // back to the last period state
                    qp_mutable_AStates.SetFrom(last_AStates); // set the current qp abstract from the last states
                    qp_mutable_AStates.get_mutable_value(11).set_value<double>(location_bias);
                    qp_mutable_AStates.get_mutable_value(14).set_value<bool>(refresh);
                    kcg.set_state_vector(&KCG_mutable_context, last_kcg_c_states);
                    sim_mutable_context.SetTime(last_c_time);
                    LastTimeCase = 0;
                    break;
                }
                case 0: { // refresh mode/*/**/*/
                    refresh = true;/**/
                    std::cout << "case0" << std::endl;
                    // to judge the searching location bias
                    if (LastTimeCase == 1){
                        if (incline == -1) {
                            bias_higher_bound = 0;
                            location_bias = (bias_lower_bound+bias_higher_bound)/2;
                        } else if (incline == 1) {
                            bias_lower_bound = 0;
                            location_bias = (bias_higher_bound+bias_lower_bound)/2;
                        }
                    } else if (LastTimeCase == 0) {
                        if (incline == -1) {
                            bias_higher_bound = location_bias;
                            location_bias = (bias_higher_bound+bias_lower_bound)/2;
                        } else if (incline == 1) {
                            bias_lower_bound = location_bias;
                            location_bias = (bias_higher_bound+bias_lower_bound)/2;
                        }
                    }
                    std::cout << "+++++++++++" << location_bias << "++++++++++++++++" << std::endl;
                    if (bias_higher_bound == bias_lower_bound){
                        std::cout << "state error!" << std::endl; exit(0);
                    }
                    auto& sim_mutable_context = simulator.get_mutable_context();
                    auto& KCG_mutable_context = diagram->GetMutableSubsystemContext(kcg, &sim_mutable_context);
                    auto& qp_mutable_context = diagram->GetMutableSubsystemContext(*qpcontroller, &sim_mutable_context);
                    auto& qp_mutable_AStates = qp_mutable_context.get_mutable_abstract_state();

                    // back to the last period state
                    qp_mutable_AStates.SetFrom(last_AStates); // set the current qp abstract from the last states
                    qp_mutable_AStates.get_mutable_value(11).set_value<double>(location_bias);
                    qp_mutable_AStates.get_mutable_value(12).set_value<bool>(true); // set as modified
                    qp_mutable_AStates.get_mutable_value(14).set_value<bool>(refresh);
                    kcg.set_state_vector(&KCG_mutable_context, last_kcg_c_states);

                    sim_mutable_context.SetTime(last_c_time);
                    LastTimeCase = 0;
                    break;
                }
            }
            std::cout << "==incline==" << incline << std::endl;
            std::cout << "simulation time:" << sim_context.get_time() << std::endl;
            simulator.Initialize();
            simulator.AdvanceTo(SIMULATION_TIME);
            if (case1_ticket) {
                case1_ticket = false;
                location_bias = 0;
                auto& tsim_context = simulator.get_context();
                auto& tkcg_context = diagram->GetSubsystemContext(kcg, tsim_context);
                auto& tqp_context = diagram->GetSubsystemContext(*qpcontroller, tsim_context);
                auto tkcg_c_states = tkcg_context.get_continuous_state().CopyToVector();
                auto tqp_d_states = tqp_context.get_discrete_state().get_vector().CopyToVector();
                auto& tqp_a_states = tqp_context.get_abstract_state();

                // current states becomes last states
                last_kcg_c_states = tkcg_c_states; // current kcg continue state  assign to last continue state
                last_qp_d_states = tqp_d_states; // current qp discrete state assign to last discrete state
                last_AStates.SetFrom(tqp_a_states);// 0 extract that reset the period_state; 12extract that set as modified

                last_c_time = tsim_context.get_time();
                LastTimeCase = 2;
                bias_higher_bound = BOUND;
                bias_lower_bound = -BOUND;
            }
        }

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

        Eigen::MatrixXd data3 = logger3->data();
        Eigen::MatrixXd sampleTime3 = logger3->sample_times();
        kFile = "data3.csv";
        writeCSV(data3, kFile);
        kFile = "time3.csv";
        writeCSV(sampleTime3, kFile);

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

//                    Abstract_vector[0] = AbstractValue::Make(qp_a_states.get_value(0).get_value<int>());
//                    Abstract_vector[1] = AbstractValue::Make(qp_a_states.get_value(1).get_value<double>());
//                    Abstract_vector[2] = AbstractValue::Make(qp_a_states.get_value(2).get_value<double>());
//                    Abstract_vector[3] = AbstractValue::Make(qp_a_states.get_value(3).get_value<Eigen::Matrix<double, NU, 1>>());
//                    Abstract_vector[4] = AbstractValue::Make(qp_a_states.get_value(4).get_value<int>());
//                    Abstract_vector[5] = AbstractValue::Make(qp_a_states.get_value(5).get_value<int>());
//                    Abstract_vector[6] = AbstractValue::Make(qp_a_states.get_value(6).get_value<bool>());
//                    Abstract_vector[7] = AbstractValue::Make(qp_a_states.get_value(7).get_value<int>());
//                    Abstract_vector[8] = AbstractValue::Make(qp_a_states.get_value(8).get_value<bool>());
//                    Abstract_vector[9] = AbstractValue::Make(qp_a_states.get_value(9).get_value<bool>());
//                    Abstract_vector[10] = AbstractValue::Make(qp_a_states.get_value(10).get_value<double>());
//                    Abstract_vector[11] = AbstractValue::Make(qp_a_states.get_value(11).get_value<double>());
//                    Abstract_vector[12] = AbstractValue::Make(qp_a_states.get_value(12).get_value<bool>());

//                    last_period_state = qp_a_states.get_value(0).get_value<int>();
//                    last_sim_time = qp_a_states.get_value(1).get_value<double>();
//                    last_t = qp_a_states.get_value(2).get_value<double>();
//                    last_t_pre = qp_a_states.get_value(3).get_value<Eigen::Matrix<double, NU, 1>>();
//                    last_m = qp_a_states.get_value(4).get_value<int>();
//                    last_count_num = qp_a_states.get_value(5).get_value<int>();
//                    last_count_num_change = qp_a_states.get_value(6).get_value<bool>();
//                    last_step_num = qp_a_states.get_value(7).get_value<int>();
//                    last_replanning = qp_a_states.get_value(8).get_value<bool>();
//                    last_isNew = qp_a_states.get_value(9).get_value<bool>();
//                    last_stance_now = qp_a_states.get_value(10).get_value<double>();
//                    last_location_bias = qp_a_states.get_value(11).get_value<double>();
// the index 12 abstract state is awalys set by main functions as a semaphore
