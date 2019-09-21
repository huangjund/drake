#include "drake/multibody/rigid_body_tree_construction.h"

namespace drake {
namespace multibody {

void AddFlatTerrainToWorld(RigidBodyTreed* tree, double box_size,
                           double box_depth) {
    double distance=10;
    double box_dep=0.1;
    double box_size1 = 0.4, box_size2 = 1;
    DrakeShapes::Box geom(Eigen::Vector3d(box_size, box_size, box_depth));
    DrakeShapes::Box geom1(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom2(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom3(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom4(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom5(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom6(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom7(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom8(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom9(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom10(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom11(Eigen::Vector3d(box_size1, box_size2, box_dep));
    DrakeShapes::Box geom12(Eigen::Vector3d(box_size1, box_size2, box_dep));

    Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link2 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link3 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link4 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link5 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link6 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link7 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link8 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link9 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link10 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link11 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_element_to_link12 = Eigen::Isometry3d::Identity();
    T_element_to_link.translation() << 0, 0, -box_depth / 2;  // Top of the box is at z = 0.
    T_element_to_link1.translation() << -box_size1/2-1.4-distance, 0, box_dep/2;
    T_element_to_link2.translation() << -3*box_size1/2-1.4-distance, 0, box_dep*3/2;
    T_element_to_link3.translation() << -5*box_size1/2-1.4-distance, 0, box_dep*5/2;
    T_element_to_link4.translation() << -7*box_size1/2-1.4-distance, 0, box_dep*7/2;
    T_element_to_link5.translation() << -9*box_size1/2-1.4-distance, 0, box_dep*9/2;
    T_element_to_link6.translation() << -11*box_size1/2-1.4-distance, 0, box_dep*11/2;
    T_element_to_link7.translation() << -13*box_size1/2-1.4-distance, 0, box_dep*13/2;
    T_element_to_link8.translation() << -15*box_size1/2-1.4-distance, 0, box_dep*15/2;
    T_element_to_link9.translation() << -17*box_size1/2-1.4-distance, 0, box_dep*17/2;
    T_element_to_link10.translation() << -19*box_size1/2-1.4-distance, 0, box_dep*19/2;
    T_element_to_link11.translation() << -21*box_size1/2-1.4-distance, 0, box_dep*21/2;
    T_element_to_link12.translation() << -23*box_size1/2-1.4-distance, 0, box_dep*23/2;
  RigidBody<double>& world = tree->world();

  // Defines a color called "desert sand" according to htmlcsscolor.com.
  Eigen::Vector4d color;
  color << 0.9297, 0.7930, 0.6758, 1;
    Eigen::Vector4d color1;
    color1 << 0.5297, 0.7930, 0.6758, 1;

  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom1, T_element_to_link1, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom2, T_element_to_link2, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom3, T_element_to_link3, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom4, T_element_to_link4, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom5, T_element_to_link5, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom6, T_element_to_link6, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom7, T_element_to_link7, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom8, T_element_to_link8, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom9, T_element_to_link9, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom10, T_element_to_link10, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom11, T_element_to_link11, color1));
    world.AddVisualElement(
            DrakeShapes::VisualElement(geom12, T_element_to_link12, color1));

    tree->addCollisionElement(
      drake::multibody::collision::Element(geom, T_element_to_link, &world),
      world, "terrain");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom1, T_element_to_link1, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom2, T_element_to_link2, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom3, T_element_to_link3, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom4, T_element_to_link4, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom5, T_element_to_link5, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom6, T_element_to_link6, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom7, T_element_to_link7, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom8, T_element_to_link8, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom9, T_element_to_link9, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom10, T_element_to_link10, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom11, T_element_to_link11, &world),
            world, "stairs");
    tree->addCollisionElement(
            drake::multibody::collision::Element(geom12, T_element_to_link12, &world),
            world, "stairs");
  tree->compile();
}

}  // namespace multibody
}  // namespace drake
