//
// Created by Junda on 7/7/19.
//

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/KneedCompassGait/KneedCompassGait.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace KneedCompassGait {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

// run drake-visualizer to view

int DoMain() {
    systems::DiagramBuilder<double> builder;

    auto tree = new RigidBodyTree<double>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            FindResourceOrThrow(
                    "drake/examples/KneedCompassGait/KneedCompassGait.urdf"),
            multibody::joints::kRollPitchYaw, tree);

    auto kneed_compass_gait = builder.AddSystem<KneedCompassGait>(tree);
    kneed_compass_gait->set_name("kneed_compass_gait");
            // add floating roll-pitch-yaw joints to unattached links
            // to all unattached links

//    {  // Add plane
//        const double box_depth = 10.0;
//        DrakeShapes::Box geom(Eigen::Vector3d(100, 1, box_depth));
//
//        // In the following use W for world frame and B for box frame.
//        Eigen::Isometry3d X_WB = Eigen::Isometry3d::Identity();
//        // Top of the box is at z = 0.
//        X_WB.translation() << 0, 0, -box_depth / 2;
//
//        // Defines a color called "desert sand" according to htmlcsscolor.com.
//        Eigen::Vector4d color(0.9297, 0.7930, 0.6758, 1);
//
//        RigidBody<double>& world = tree->world();
//        world.AddVisualElement(DrakeShapes::VisualElement(geom, X_WB, color));
//        tree->addCollisionElement(
//             drake::multibody::collision::Element(geom, X_WB, &world), world,
//                "terrain");
//        tree->compile();
//    }

    lcm::DrakeLcm lcm;
    auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
    publisher->set_name("publisher");

    builder.Connect(kneed_compass_gait->get_output_port(1),
                    publisher->get_input_port(0));
    auto diagram = builder.Build();

    systems::Simulator<double> simulator(*diagram);
    systems::Context<double>& rw_context = diagram->GetMutableSubsystemContext(
            *kneed_compass_gait, &simulator.get_mutable_context());
    KneedcompassgaitContinuousstate<double>& state =
            kneed_compass_gait->get_mutable_continuous_state(&rw_context);

    state.set_angle_stance_knee(2.0);
    state.set_angle_swing_knee(0.0);
    state.set_angle_hip(0.0);
    state.set_angle_thigh(0.0);
    state.set_angledot_stance_knee(0.0);
    state.set_angledot_swing_knee(0.0);
    state.set_angledot_hip(0.0);
    state.set_angledot_thigh(0.0);

    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.get_mutable_context().SetAccuracy(1e-4);
   // std::cout << "ok" << std::endl;
    simulator.AdvanceTo(10);
    return 0;
}

}  // namespace
}  // namespace KneedCompassGait
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::examples::KneedCompassGait::DoMain();
}
