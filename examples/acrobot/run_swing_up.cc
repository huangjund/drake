#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_state.h"
#include "drake/examples/acrobot/spong_controller.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/wrap_to.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

// Simple example which simulates the Acrobot, started near its stable fixed
// point, with a Spong swing-up controller designed to reach the unstable
// fixed point.  Run drake-visualizer to see the animated result.

DEFINE_double(simulation_sec, 10.0,
              "Number of seconds to simulate.");
DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
int writeCSV(Eigen::MatrixXd data, std::string& kFile) {
    std::ofstream outFile;
    outFile.open(kFile, std::ios::out); // 打开模式可省略
    outFile << data << std::endl;
    outFile.close();

    return 1;
}
int do_main() {
    using std::cout;
    using std::endl;

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"),
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto acrobot = builder.AddSystem<AcrobotPlant>();
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(acrobot->get_output_port(0), publisher->get_input_port(0));

  auto controller = builder.AddSystem<AcrobotSpongController>();
  builder.Connect(acrobot->get_output_port(0), controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0), acrobot->get_input_port(0));

  auto logger = systems::LogOutput(acrobot->get_output_port(0), &builder);
  auto logger2 = systems::LogOutput(controller->get_output_port(0), &builder);

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& acrobot_context =
      diagram->GetMutableSubsystemContext(*acrobot,
                                          &simulator.get_mutable_context());

  // Sets an initial condition near the upright fixed point.
  AcrobotState<double>* state = dynamic_cast<AcrobotState<double>*>(
      &acrobot_context.get_mutable_continuous_state_vector());
  DRAKE_DEMAND(state != nullptr);
  state->set_theta1(0.1);
  state->set_theta2(-0.1);
  state->set_theta1dot(0.0);
  state->set_theta2dot(0.02);

  simulator.set_publish_every_time_step(1);
  simulator.set_target_realtime_rate(10);
  simulator.AdvanceTo(FLAGS_simulation_sec);

  cout << "realtime\n" << simulator.get_actual_realtime_rate() << endl;
  DRAKE_DEMAND(std::abs(math::wrap_to(state->theta1(), 0., 2. * M_PI) - M_PI) <
               1e-2);
  DRAKE_DEMAND(std::abs(math::wrap_to(state->theta2(), -M_PI, M_PI)) < 1e-2);
  DRAKE_DEMAND(std::abs(state->theta1dot()) < 0.1);
  DRAKE_DEMAND(std::abs(state->theta2dot()) < 0.1);

  const systems::Context<double>& context_acrobot =
          diagram->GetSubsystemContext(*acrobot, simulator.get_context());
  const systems::Context<double>& context_controller =
          diagram->GetSubsystemContext(*controller, simulator.get_context());

  cout << "time\n" << context_acrobot.get_time() << endl;
  cout << "accuracy\n" << context_acrobot.get_accuracy().has_value() << endl;

  cout << "controller time\n" << context_controller.get_time() << endl;
  cout << "controller accuracy\n" << context_controller.get_accuracy().has_value() << endl;

  Eigen::MatrixXd data = logger->data();
  Eigen::MatrixXd sampleTime = logger->sample_times();
  std::string kFile;
  kFile = "data.csv";
  writeCSV(data, kFile);
  kFile = "time.csv";
  writeCSV(sampleTime, kFile);
    Eigen::MatrixXd data1 = logger2->data();
    Eigen::MatrixXd sampleTime1 = logger2->sample_times();
    kFile = "data1.csv";
    writeCSV(data1, kFile);
    kFile = "time1.csv";
    writeCSV(sampleTime1, kFile);
  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::acrobot::do_main();
}
