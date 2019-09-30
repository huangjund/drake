//
// Created by Junda Huang on 9/29/19.
//
#include "drake/examples/kuka_iiwa_lidar/includes/AddTerrainToWorld.h"

namespace drake {
namespace examples {
namespace Terrain {
    void AddBasicTerrain(RigidBodyTree<double>* tree, double width, double length, double depth) {
        std::string kFilePath;
        double radius = 0.1;
        double len = 0.5;
        Eigen::Vector4d color1;
        color1 << 0.9297, 0.7930, 0.6758, 1;
        Eigen::Vector4d FireBrick;
        FireBrick << 0.7236, 0.1382, 0.1382, 1;
        Eigen::Vector4d lucency;
        lucency << 0.3423, 0.3356, 0.3221, 0.5;
        Eigen::Vector4d white;
        white << 0.32, 0.34, 0.34, 1;
        Eigen::Vector4d black;
        black << 0, 0, 0, 1;


        DrakeShapes::Box Ground(Eigen::Vector3d(width, length, depth));
        DrakeShapes::Cylinder cylinder1(radius,len);
        DrakeShapes::Cylinder cylinder2(radius,len);
        DrakeShapes::Box belt1(Eigen::Vector3d(len, 2, 0.001));
        DrakeShapes::Box belt2(Eigen::Vector3d(len, 2, 0.001));


        kFilePath = "drake/examples/kuka_iiwa_lidar/models/Objects/table.obj";
        DrakeShapes::Mesh table1("table1", drake::FindResourceOrThrow(kFilePath));
        DrakeShapes::Mesh table2("table2", drake::FindResourceOrThrow(kFilePath));
        DrakeShapes::Mesh table3("table3", drake::FindResourceOrThrow(kFilePath));
        DrakeShapes::Mesh table4("table4", drake::FindResourceOrThrow(kFilePath));
        DrakeShapes::Mesh table5("table5", drake::FindResourceOrThrow(kFilePath));
        kFilePath = "drake/examples/kuka_iiwa_lidar/models/Objects/mug.obj";
        DrakeShapes::Mesh mug1("mug1", drake::FindResourceOrThrow(kFilePath));
        DrakeShapes::Mesh mug2("mug2", drake::FindResourceOrThrow(kFilePath));
        kFilePath = "drake/examples/kuka_iiwa_lidar/models/Objects/cup.obj";
        DrakeShapes::Mesh cup1("cup1", drake::FindResourceOrThrow(kFilePath));
        DrakeShapes::Mesh cup2("cup2", drake::FindResourceOrThrow(kFilePath));
        DrakeShapes::Mesh cup3("cup4", drake::FindResourceOrThrow(kFilePath));
        DrakeShapes::Mesh cup4("cup4", drake::FindResourceOrThrow(kFilePath));

        Eigen::Isometry3d Transform_Ground = Eigen::Isometry3d::Identity();
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

        Transform_Ground.translation() << 0, 0, -depth/2 - 0.7; // set the ground 0.7is the height of the table
        Transform_cylinder1.translation() << 1, 1, 0;
        Transform_cylinder2.translation() << 1, -1, 0;
        Transform_belt1.translation() << 1, 0, radius;
        Transform_belt2.translation() << 1, 0, -radius;
        Transform_table1.translation() << 0, 0, -0.1;
        Transform_table2.translation() << -1, 0.4, -0.1;
        Transform_table3.translation() << 0, 1, -0.1;
        Transform_table4.translation() << 0, -0.7, -0.1;
        Transform_table5.translation() << -1, -0.7, -0.1;
        Transform_mug1.translation() << -0.1, 0.9, 0;
        Transform_mug2.translation() << -1, 0.4, 0;
        Transform_cup1.translation() << 0.1, 1.1, -0.1;
        Transform_cup2.translation() << 0, -0.7, -0.1;
        Transform_cup3.translation() << 1, 0.2, 0.1;
        Transform_cup4.translation() << 1, -0.7, 0.1;

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


        RigidBody<double>& world = tree->world();

        world.AddVisualElement(
                DrakeShapes::VisualElement(Ground, Transform_Ground, color1));
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
        world.AddVisualElement(
                DrakeShapes::VisualElement(mug2, Transform_mug2, lucency));
        world.AddVisualElement(
                DrakeShapes::VisualElement(cup1, Transform_cup1, lucency));
        world.AddVisualElement(
                DrakeShapes::VisualElement(cup2, Transform_cup2, lucency));
        world.AddVisualElement(
                DrakeShapes::VisualElement(cup3, Transform_cup3, white));
        world.AddVisualElement(
                DrakeShapes::VisualElement(cup4, Transform_cup4, white));

        tree->addCollisionElement(
                drake::multibody::collision::Element(Ground, Transform_Ground, &world), world, "basicground");
        tree->addCollisionElement(
                drake::multibody::collision::Element(table1, Transform_table1, &world), world, "table1");
        tree->addCollisionElement(
                drake::multibody::collision::Element(table2, Transform_table2, &world), world, "table2");
        tree->addCollisionElement(
                drake::multibody::collision::Element(table3, Transform_table3, &world), world, "table3");
        tree->addCollisionElement(
                drake::multibody::collision::Element(table4, Transform_table4, &world), world, "table4");
        tree->addCollisionElement(
                drake::multibody::collision::Element(table5, Transform_table5, &world), world, "table5");
        tree->addCollisionElement(
                drake::multibody::collision::Element(mug1, Transform_mug1, &world), world, "mug1");
        tree->addCollisionElement(
                drake::multibody::collision::Element(mug2, Transform_mug2, &world), world, "mug2");
        tree->addCollisionElement(
                drake::multibody::collision::Element(cup1, Transform_cup1, &world), world, "cup1");
        tree->addCollisionElement(
                drake::multibody::collision::Element(cup2, Transform_cup2, &world), world, "cup2");

        tree->compile();
    }
} // namespace Terrain
} // namespace examples
} // namespace drake
