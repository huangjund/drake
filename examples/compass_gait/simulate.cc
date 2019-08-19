#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/compass_gait/compass_gait.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace compass_gait {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

/// Simulates the compass gait from various initial velocities (accepted as
/// command-line arguments.  Run drake-visualizer to watch the results.
int DoMain() {
  systems::DiagramBuilder<double> builder;
//  std::cout << "1" << std::endl;
  auto compass_gait = builder.AddSystem<CompassGait>();
//  std::cout << "2" << std::endl;
  compass_gait->set_name("compass_gait");

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/compass_gait/CompassGait.urdf"),
      multibody::joints::kRollPitchYaw, tree.get());
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            FindResourceOrThrow("drake/examples/compass_gait/quadrotor.urdf"),
            multibody::joints::kFixed, tree.get());
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                FindResourceOrThrow("drake/examples/compass_gait/quadrotor1.urdf"),
                multibody::joints::kFixed, tree.get());
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                FindResourceOrThrow("drake/examples/compass_gait/quadrotor2.urdf"),
                multibody::joints::kFixed, tree.get());
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                FindResourceOrThrow("drake/examples/compass_gait/quadrotor3.urdf"),
                multibody::joints::kFixed, tree.get());
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                FindResourceOrThrow("drake/examples/compass_gait/quadrotor4.urdf"),
                multibody::joints::kFixed, tree.get());
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            FindResourceOrThrow("drake/examples/compass_gait/cassie.urdf"),
            multibody::joints::kFixed, tree.get());


  {  // Add ramp
    // TODO(russt): Consider moving/reusing this block (useful for all passive
    // walkers).
   // const double box_depth = 10.0;
    std::string kFilePath;
    kFilePath = "drake/examples/compass_gait/simulator1.obj";
    DrakeShapes::Mesh geom("scenario1", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/table.obj";
      DrakeShapes::Mesh geom1("scenario2", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/table1.obj";
      DrakeShapes::Mesh geom2("scenario3", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/table2.obj";
      DrakeShapes::Mesh geom3("scenario4", drake::FindResourceOrThrow(kFilePath));


    // In the following use W for world frame and B for box frame.

    Eigen::Isometry3d X_WB = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd rotation_base(M_PI/2, Vector3<double>(1, 0, 0));
    X_WB.rotate(rotation_base);
    X_WB.translation() << -1, -5, 1; // -box_depth / 2;  Top of the box is at z = 0.

      Eigen::Isometry3d X_WB1 = Eigen::Isometry3d::Identity();
      Eigen::AngleAxisd rotation_base1(M_PI/2, Vector3<double>(1, 0, 0));
      X_WB1.rotate(rotation_base1);
      X_WB1.translation() << 12, -7.2, 2.3; // -box_depth / 2;  Top of the box is at z = 0.

      Eigen::Isometry3d X_WB2 = Eigen::Isometry3d::Identity();
      Eigen::AngleAxisd rotation_base2(M_PI/2, Vector3<double>(1, 0, 0));
      X_WB2.rotate(rotation_base2);
      X_WB2.translation() << 3.5, -8.5, 1.9; // -box_depth / 2;  Top of the box is at z = 0.

      Eigen::Isometry3d X_WB3 = Eigen::Isometry3d::Identity();
      Eigen::AngleAxisd rotation_base3(M_PI/2, Vector3<double>(1, 0, 0));
      X_WB3.rotate(rotation_base3);
      X_WB3.translation() << 7, -11, 1.5; // -box_depth / 2;  Top of the box is at z = 0.
   // double ramp_pitch = CompassGaitParams<double>().slope();
   // X_WB.rotate(
   //    math::RotationMatrix<double>::MakeYRotation(ramp_pitch).matrix());

    // Defines a color called "desert sand" according to htmlcsscolor.com.
    Eigen::Vector4d color1(0.9297, 0.7930, 0.6758, 1);
      Eigen::Vector4d color2(0.7236, 0.1382, 0.1382, 1);


    RigidBody<double>& world = tree->world();
    world.AddVisualElement(DrakeShapes::VisualElement(geom, X_WB, color1));
      world.AddVisualElement(DrakeShapes::VisualElement(geom1, X_WB1, color2));
      world.AddVisualElement(DrakeShapes::VisualElement(geom2, X_WB2, color2));
      world.AddVisualElement(DrakeShapes::VisualElement(geom3, X_WB3, color2));
    tree->addCollisionElement(
        drake::multibody::collision::Element(geom, X_WB, &world), world,
        "terrain");
    tree->compile();
  }

  lcm::DrakeLcm lcm;
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");

  builder.Connect(compass_gait->get_floating_base_state_output_port(),
                  publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& cg_context = diagram->GetMutableSubsystemContext(
      *compass_gait, &simulator.get_mutable_context());
  CompassGaitContinuousState<double>& state =
      compass_gait->get_mutable_continuous_state(&cg_context);
  state.set_stance(0.0);
  state.set_swing(0.0);
  state.set_stancedot(0.4);
  state.set_swingdot(-2.0);
  compass_gait->get_input_port(0).FixValue(&cg_context, Vector1d(0.0));

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.get_mutable_context().SetAccuracy(1e-4);
 // std::cout << "before simulate" << std::endl;
  simulator.AdvanceTo(5);

  return 0;
}

}  // namespace
}  // namespace compass_gait
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::compass_gait::DoMain();
}
