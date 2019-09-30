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
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsers/sdf_parser.h"

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
        using manipulation::util::SimDiagramBuilder;
        using systems::DiagramBuilder;
        using systems::Simulator;
        using multibody::MultibodyPlant;
        using systems::RigidBodyPlant;
        using std::cout;
        using std::endl;

        double bias_x=-9, bias_y=-6, bias_z=-2.7;

        drake::lcm::DrakeLcm lcm;
        SimDiagramBuilder<double> builder;
//        RigidBodyPlant<double>* plant = nullptr;

        // Create Plant
        auto tree = std::make_unique<RigidBodyTree<double>>();

        std::string filepath;
        filepath = "drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf";
        drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                FindResourceOrThrow(filepath), multibody::joints::kFixed, tree.get());

        drake::parsers::sdf::AddModelInstancesFromSdfFile(
                FindResourceOrThrow(
                        "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf"),
                kFixed, nullptr /* weld to frame */, rigid_body_tree.get());

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
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                FindResourceOrThrow("drake/examples/compass_gait/cart.urdf"),
                multibody::joints::kFixed, tree.get());
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                FindResourceOrThrow("drake/examples/compass_gait/cart1.urdf"),
                multibody::joints::kFixed, tree.get());
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                FindResourceOrThrow("drake/examples/compass_gait/cart2.urdf"),
                multibody::joints::kFixed, tree.get());


  {
      double radius = 0.1;
      double len = 0.5;
      Eigen::Vector4d color1(0.9297, 0.7930, 0.6758, 1);
      Eigen::Vector4d color2(0.7236, 0.1382, 0.1382, 1);
      Eigen::Vector4d color3(0.3423, 0.3356, 0.3221, 0.5);
      Eigen::Vector4d FireBrick;
      FireBrick << 0.7236, 0.1382, 0.1382, 1;
      Eigen::Vector4d lucency;
      lucency << 0.3423, 0.3356, 0.3221, 0.5;
      Eigen::Vector4d white;
      white << 0.32, 0.34, 0.34, 1;
      Eigen::Vector4d black;
      black << 0, 0, 0, 1;
      Eigen::Vector4d blue;
      blue << 0, 0, 1, 1;
      Eigen::Vector4d green;
      green << 0, 1, 0, 1;


      DrakeShapes::Cylinder cylinder1(radius,len);
      DrakeShapes::Cylinder cylinder2(radius,len);
      DrakeShapes::Box belt1(Eigen::Vector3d(len, 2, 0.001));
      DrakeShapes::Box belt2(Eigen::Vector3d(len, 2, 0.001));
      DrakeShapes::Sphere circle(radius);
      DrakeShapes::Cylinder hammer(radius/2, 0.2);
      DrakeShapes::Box box(Eigen::Vector3d(0.1, 0.1, 0.1));

      std::string kFilePath;

      kFilePath = "drake/examples/compass_gait/table.obj";
      DrakeShapes::Mesh table1("table1", drake::FindResourceOrThrow(kFilePath));
      DrakeShapes::Mesh table2("table2", drake::FindResourceOrThrow(kFilePath));
      DrakeShapes::Mesh table3("table3", drake::FindResourceOrThrow(kFilePath));
      DrakeShapes::Mesh table4("table4", drake::FindResourceOrThrow(kFilePath));
      DrakeShapes::Mesh table5("table5", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/mug1.obj";
      DrakeShapes::Mesh mug1("mug1", drake::FindResourceOrThrow(kFilePath));
      DrakeShapes::Mesh mug2("mug2", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/cup1.obj";
      DrakeShapes::Mesh cup1("cup1", drake::FindResourceOrThrow(kFilePath));
      DrakeShapes::Mesh cup2("cup2", drake::FindResourceOrThrow(kFilePath));
      DrakeShapes::Mesh cup3("cup4", drake::FindResourceOrThrow(kFilePath));
      DrakeShapes::Mesh cup4("cup4", drake::FindResourceOrThrow(kFilePath));


    kFilePath = "drake/examples/compass_gait/simulator1.obj";
    DrakeShapes::Mesh geom("scenario1", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/table.obj";
      DrakeShapes::Mesh geom1("scenario2", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/table1.obj";
      DrakeShapes::Mesh geom2("scenario3", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/table2.obj";
      DrakeShapes::Mesh geom3("scenario4", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/mug1.obj";
      DrakeShapes::Mesh geom4("scenario5", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/cup1.obj";
      DrakeShapes::Mesh geom5("scenario6", drake::FindResourceOrThrow(kFilePath));
      kFilePath = "drake/examples/compass_gait/mug1.obj";
      DrakeShapes::Mesh geom6("scenario5", drake::FindResourceOrThrow(kFilePath));

      Eigen::Isometry3d Transform_cylinder1 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_cylinder2 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_belt1 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_belt2 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_table1 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_table2 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_table3 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_table4 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_table5 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_mug1 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_mug2 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_cup1 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_cup2 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_cup3 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_cup4 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_circle = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_hammer = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Transform_box = Eigen::Isometry3d::Identity();

      Transform_cylinder1.translation() << 1, 1, 0;
      Transform_cylinder2.translation() << 1, -1, 0;
      Transform_belt1.translation() << 1, 0, radius;
      Transform_belt2.translation() << 1, 0, -radius;
      Transform_table1.translation() << 0, 0, -0.1;
      Transform_table2.translation() << 1, 0.8, -0.2;
      Transform_mug2.translation() << -1, 0.4, 0;
      Transform_table3.translation() << 0, 1, -0.1;
      Transform_table4.translation() << -1, -0.7, -0.1;
      Transform_cup2.translation() << -1, -0.7, -0.1;
      Transform_table5.translation() << 1, -0.8, -0.2;
      Transform_mug1.translation() << -0.1, 0.9, 0;
      Transform_cup1.translation() << 0.1, 1.1, -0.1;
      Transform_cup3.translation() << 1, 0.2, 0.1;
      Transform_cup4.translation() << 1, -0.7, 0.1;
      Transform_circle.translation() << 1, -0.8, 0.2;
      Transform_hammer.translation() << 1, 0, 0.2;
      Transform_box.translation() << 1, 0.8, 0.2;

      Eigen::AngleAxisd rotation_base(M_PI/2, Vector3<double>(1, 0, 0));
      Transform_table1.rotate(rotation_base);
      Transform_table2.rotate(rotation_base);
      Transform_table3.rotate(rotation_base);
      Transform_table4.rotate(rotation_base);
      Transform_table5.rotate(rotation_base);
      Transform_mug1.rotate(rotation_base);
      Transform_mug2.rotate(rotation_base);
      Transform_cup1.rotate(rotation_base);
      Transform_cup2.rotate(rotation_base);
      Transform_cup3.rotate(rotation_base);

      Eigen::AngleAxisd rotation_cylinder(M_PI/2, Vector3<double>(0, 1, 0));
      Transform_cylinder1.rotate(rotation_cylinder);
      Transform_cylinder2.rotate(rotation_cylinder);
      Transform_cup4.rotate(rotation_cylinder);
    // In the following use W for world frame and B for box frame.

    Eigen::Isometry3d X_WB = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd rotation_base7(M_PI/2, Vector3<double>(1, 0, 0));
    X_WB.rotate(rotation_base7);
    X_WB.translation() << -10+bias_x, 10+bias_y, 1+bias_z; // -box_depth / 2;  Top of the box is at z = 0.

      Eigen::Isometry3d X_WB1 = Eigen::Isometry3d::Identity();
      Eigen::AngleAxisd rotation_base1(M_PI/2, Vector3<double>(1, 0, 0));
      X_WB1.rotate(rotation_base1);
      X_WB1.translation() << 8+bias_x, 6+bias_y, 2.4+bias_z; // -box_depth / 2;  Top of the box is at z = 0.

      Eigen::Isometry3d X_WB2 = Eigen::Isometry3d::Identity();
      Eigen::AngleAxisd rotation_base2(M_PI/2, Vector3<double>(1, 0, 0));
      X_WB2.rotate(rotation_base2);
      X_WB2.translation() << 1.5+bias_x, 4+bias_y, 2.4+bias_z; // -box_depth / 2;  Top of the box is at z = 0.

      Eigen::Isometry3d X_WB3 = Eigen::Isometry3d::Identity();
      Eigen::AngleAxisd rotation_base3(M_PI/2, Vector3<double>(1, 0, 0));
      X_WB3.rotate(rotation_base3);
      X_WB3.translation() << 7+bias_x, -2+bias_y, 1.5+bias_z; // -box_depth / 2;  Top of the box is at z = 0.

      Eigen::Isometry3d X_WB4 = Eigen::Isometry3d::Identity();
      Eigen::AngleAxisd rotation_base4(M_PI/2, Vector3<double>(1, 0, 0));
      X_WB4.rotate(rotation_base4);
      X_WB4.translation() << 8+bias_x, 6.2+bias_y, 2.5+bias_z; // -box_depth / 2;  Top of the box is at z = 0.

      Eigen::Isometry3d X_WB5 = Eigen::Isometry3d::Identity();
      Eigen::AngleAxisd rotation_base5(M_PI/2, Vector3<double>(1, 0, 0));
      X_WB5.rotate(rotation_base5);
      X_WB5.translation() << 8.2+bias_x, 6+bias_y, 2.5+bias_z; // -box_depth / 2;  Top of the box is at z = 0.

      Eigen::Isometry3d X_WB6 = Eigen::Isometry3d::Identity();
      Eigen::AngleAxisd rotation_base6(M_PI/2, Vector3<double>(1, 0, 0));
      X_WB6.rotate(rotation_base6);
      X_WB6.translation() << 1.5+bias_x, 4+bias_y, 2.5+bias_z; // -box_depth / 2;  Top of the box is at z = 0.



    RigidBody<double>& world = tree->world();

    world.AddVisualElement(DrakeShapes::VisualElement(geom, X_WB, color1));
      world.AddVisualElement(DrakeShapes::VisualElement(geom1, X_WB1, color2));
      world.AddVisualElement(DrakeShapes::VisualElement(geom2, X_WB2, color2));
      world.AddVisualElement(DrakeShapes::VisualElement(geom3, X_WB3, color2));
      world.AddVisualElement(DrakeShapes::VisualElement(geom4, X_WB4, color3));
      world.AddVisualElement(DrakeShapes::VisualElement(geom5, X_WB5, color3));
      world.AddVisualElement(DrakeShapes::VisualElement(geom6, X_WB6, color3));

      world.AddVisualElement(
              DrakeShapes::VisualElement(cylinder1, Transform_cylinder1, white));
      world.AddVisualElement(
              DrakeShapes::VisualElement(cylinder2, Transform_cylinder2, white));
      world.AddVisualElement(
              DrakeShapes::VisualElement(belt1, Transform_belt1, black));
      world.AddVisualElement(
              DrakeShapes::VisualElement(belt2, Transform_belt2, black));
      world.AddVisualElement(
              DrakeShapes::VisualElement(table1, Transform_table1, FireBrick));
      world.AddVisualElement(
              DrakeShapes::VisualElement(table2, Transform_table2, FireBrick));
      world.AddVisualElement(
              DrakeShapes::VisualElement(table3, Transform_table3, FireBrick));
      world.AddVisualElement(
              DrakeShapes::VisualElement(table4, Transform_table4, FireBrick));
      world.AddVisualElement(
              DrakeShapes::VisualElement(table5, Transform_table5, FireBrick));
      world.AddVisualElement(
              DrakeShapes::VisualElement(mug1, Transform_mug1, lucency));
//      world.AddVisualElement(
//              DrakeShapes::VisualElement(mug2, Transform_mug2, lucency));
      world.AddVisualElement(
              DrakeShapes::VisualElement(cup1, Transform_cup1, lucency));
      world.AddVisualElement(
              DrakeShapes::VisualElement(cup2, Transform_cup2, lucency));
//      world.AddVisualElement(
//              DrakeShapes::VisualElement(cup3, Transform_cup3, white));
//      world.AddVisualElement(
//              DrakeShapes::VisualElement(cup4, Transform_cup4, white));
    world.AddVisualElement(
            DrakeShapes::VisualElement(circle, Transform_circle, green));
    world.AddVisualElement(
            DrakeShapes::VisualElement(hammer, Transform_hammer, blue));
    world.AddVisualElement(
            DrakeShapes::VisualElement(box, Transform_box, color1));

    tree->addCollisionElement(
        drake::multibody::collision::Element(geom, X_WB, &world), world,
        "terrain");
    tree->compile();
  }
        RigidBodyPlant<double>* plant = nullptr;
        plant = builder.AddPlant(std::move(tree));

        // Create LCM publisher for visualization
        builder.AddVisualizer(&lcm);
        builder.get_visualizer()->set_publish_period(1e-2);

        auto sys = builder.Build();

        Simulator<double> simulator(*sys);

        cout << plant->get_num_states() << endl;
        Eigen::Matrix<double, 14, 1> PointState;
        PointState << 2.14, -1.5, -1.57, -1.57, 1.57, -1, 0,
                    0, 0, 0, 0, 0, 0, 0;

        plant->set_state_vector(&simulator.get_mutable_context(),
                                PointState);

        simulator.set_publish_every_time_step(true);
        simulator.set_target_realtime_rate(0.1);
        simulator.get_mutable_context().SetAccuracy(1e-4);

        simulator.Initialize();
        simulator.AdvanceTo(0.25);

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
